"""
AI Agent Module for Robot Voice Pipeline

Uses LangChain with OpenAI to process user queries and generate responses.
Integrates RAG (Retrieval-Augmented Generation) for knowledge-based responses.
"""

import os
import random
from typing import Optional
from collections import OrderedDict

from langchain_openai import ChatOpenAI
from langchain_core.messages import HumanMessage, SystemMessage

try:
    from robot_pipeline.ai.prompts import PromptGenerator
except ImportError:
    PromptGenerator = None

try:
    from robot_pipeline.ai.rag_database import RAGDatabase
except ImportError:
    RAGDatabase = None

try:
    from robot_pipeline.ai.faq_database import FAQDatabase
except ImportError:
    FAQDatabase = None

try:
    from robot_pipeline.ai.web_search import WebSearch
except ImportError:
    WebSearch = None


class AIAgent:
    """
    AI Agent for processing user queries and generating responses.
    
    Uses OpenAI's GPT models via LangChain with RAG (Retrieval-Augmented Generation)
    to provide knowledge-based responses using documents from the vector database.
    """
    
    def __init__(
        self,
        model: str = "gpt-4o-mini",
        api_key: Optional[str] = None,
        prompt_generator: Optional[object] = None,
        rag_database: Optional[object] = None,
        faq_database: Optional[object] = None,
        temperature: float = 0.7,
        use_rag: bool = True,
        use_faq: bool = True,
        use_web_search: bool = True,
    ):
        """
        Initialize the AI agent.
        
        Args:
            model: OpenAI model to use (default: gpt-4o-mini)
            api_key: OpenAI API key (reads from OPENAI_API_KEY env var if not provided)
            prompt_generator: PromptGenerator instance for dynamic prompts
            rag_database: RAGDatabase instance for document retrieval
            faq_database: FAQDatabase instance for fast FAQ responses
            temperature: Sampling temperature (0.0 to 1.0)
            use_rag: Whether to use RAG for context retrieval
            use_faq: Whether to check FAQ database first
            use_web_search: Whether to use Tavily web search for general questions
        """
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")
        
        self.llm = ChatOpenAI(
            model=model,
            api_key=self.api_key,
            temperature=0.1,  # Even lower for faster, more focused responses
            streaming=True,  # Enable streaming for real-time responses
            max_tokens=100,  # Increased from 60 - allows 2-3 sentences with details
        )
        
        self.prompt_generator = prompt_generator
        self.rag_database = rag_database
        self.faq_database = faq_database
        self.use_rag = use_rag and rag_database is not None
        self.use_faq = use_faq and faq_database is not None
        
        # Initialize Tavily web search
        self.web_search = None
        self.use_web_search = use_web_search
        if use_web_search and WebSearch:
            try:
                self.web_search = WebSearch()
            except Exception as e:
                print(f"Web search initialization failed: {e}")
                self.use_web_search = False
        
        self.conversation_history = []
        
        # Enhanced LRU cache for RAG contexts (speeds up repeated queries)
        self._rag_cache = OrderedDict()
        self._max_cache_size = 100  # Increased from 20 for better hit rate
        
        if self.use_faq:
            print("FAQ enabled - Fast responses for common questions")
        if self.use_rag:
            print("RAG enabled - AI will use knowledge base for responses")
        else:
            print("RAG disabled - AI will use only training knowledge")
        if self.use_web_search:
            print("Web search enabled (Tavily) - Can answer current events")
        
        # Department-related keywords for smart RAG filtering
        self.department_keywords = [
            "department", "entc", "electronic", "telecommunication", "biomedical",
            "university", "moratuwa", "professor", "dr ", "lecturer", "staff",
            "lab", "laboratory", "research", "facility", "program", "course",
            "degree", "bachelor", "master", "phd", "undergraduate", "graduate",
            "head", "hod", "office", "location", "where is", "take me",
            "computer lab", "conference", "building", "room", "floor",
            "club", "student", "admission", "enroll", "study", "learn"
        ]
    
    def _needs_department_knowledge(self, query: str) -> bool:
        """
        Determine if a query needs department-specific knowledge from RAG.
        Returns False for general questions (time, date, greetings, etc.)
        Returns True for department-related questions.
        """
        query_lower = query.lower()
        
        # Skip RAG for simple general queries
        general_patterns = [
            "time", "date", "today", "now", "current",
            "hello", "hi", "hey", "good morning", "good afternoon",
            "how are you", "thank you", "thanks", "bye", "goodbye",
            "what is your name", "who are you", "what can you do"
        ]
        
        # If it's a short general query, don't use RAG
        if any(pattern in query_lower for pattern in general_patterns):
            # But if it also contains department keywords, still use RAG
            if not any(keyword in query_lower for keyword in self.department_keywords):
                return False
        
        # Default to using RAG for anything that might be department-related
        return True
    
    def _normalize_query(self, query: str) -> str:
        """Normalize query for better cache hits."""
        import string
        # Remove punctuation, extra whitespace, convert to lowercase
        query = query.lower().strip()
        query = query.translate(str.maketrans('', '', string.punctuation))
        query = ' '.join(query.split())  # Normalize whitespace
        return query
    
    def _build_system_prompt(self, user_input: str, skip_rag: bool = False, web_context: str = "") -> str:
        """Build dynamic system prompt using PromptGenerator, RAG context, and web search."""
        # Retrieve relevant context from RAG if enabled
        rag_context = ""
        if not skip_rag and self.use_rag and self.rag_database:
            # Check cache first with normalized query
            cache_key = self._normalize_query(user_input)
            if cache_key in self._rag_cache:
                rag_context = self._rag_cache[cache_key]
                # Move to end (most recently used)
                self._rag_cache.move_to_end(cache_key)
            else:
                # Retrieve from RAG
                rag_context = self.rag_database.get_context_string(
                    query=user_input,
                    k=3,  # Retrieve top 2 relevant chunks (faster, sufficient for most queries)
                    max_length=1500  # Increased from 800 to fit 2 full chunks
                )
                
                # Cache the result
                self._rag_cache[cache_key] = rag_context
                # Limit cache size
                if len(self._rag_cache) > self._max_cache_size:
                    self._rag_cache.popitem(last=False)  # Remove oldest
        
        # Merge web context and RAG context
        context = ""
        if web_context and rag_context:
            context = web_context + "\n\n" + rag_context
        elif web_context:
            context = web_context
        elif rag_context:
            context = rag_context
        
        if self.prompt_generator:
            # Build chat history string
            chat_history = ""
            for msg in self.conversation_history[-2:]:  # Reduced from 4 to 2 (last exchange only)
                if isinstance(msg, HumanMessage):
                    chat_history += f"User: {msg.content}\n"
                else:
                    chat_history += f"Robot: {msg.content}\n"
            
            return self.prompt_generator._base_prompt(
                user="Visitor",
                user_input=user_input,
                chat_history=chat_history,
                instructions="",
                context=context
            )
        else:
            # Default prompt with context
            base_prompt = """You are a helpful robot assistant. You can answer questions, 
provide information, and assist with various tasks. Be concise and friendly 
in your responses. Keep your answers relatively short since they will be 
converted to speech."""
            
            if context:
                return f"{base_prompt}\n\n{context}"
            return base_prompt
    
    async def think(self, user_input: str) -> str:
        """
        Process user input and generate a response.
        Uses FAQ → RAG → LLM pipeline for optimal speed and accuracy.
        
        Args:
            user_input: The user's query or statement
        
        Returns:
            The AI agent's response text
        """
        # Step 1: Try FAQ first (instant, no API cost)
        if self.use_faq and self.faq_database:
            faq_answer = self.faq_database.get_answer(user_input)
            if faq_answer:
                print(f"FAQ Hit! Instant response")
                # Update conversation history
                self.conversation_history.append(HumanMessage(content=user_input))
                from langchain_core.messages import AIMessage
                self.conversation_history.append(AIMessage(content=faq_answer))
                
                # Keep history manageable
                if len(self.conversation_history) > 6:
                    self.conversation_history = self.conversation_history[-6:]
                
                return faq_answer
        
        # Step 2: Check if query needs web search
        web_context = ""
        if self.use_web_search and self.web_search:
            if self.web_search.needs_web_search(user_input):
                web_context = self.web_search.search(user_input, max_results=1)  # Reduced from 2 for speed
        
        # Step 3: Check if query needs department knowledge
        needs_rag = self._needs_department_knowledge(user_input)
        
        if not needs_rag and not web_context:
            print(f"Simple query detected - Skipping RAG for faster response")
        
        # Step 4: Build system prompt (with RAG and/or web search)
        system_prompt = self._build_system_prompt(user_input, skip_rag=not needs_rag, web_context=web_context)
        
        # Build messages for the LLM
        messages = [SystemMessage(content=system_prompt)]
        
        # Add conversation history
        messages.extend(self.conversation_history)
        
        # Add current user input
        messages.append(HumanMessage(content=user_input))
        
        # Get response from LLM
        print(f"Thinking about: '{user_input}'")
        
        try:
            response = await self.llm.ainvoke(messages)
            response_text = response.content
            
            # Update conversation history
            self.conversation_history.append(HumanMessage(content=user_input))
            self.conversation_history.append(response)
            
            # Keep conversation history manageable (last 6 messages = 3 exchanges)
            if len(self.conversation_history) > 6:
                self.conversation_history = self.conversation_history[-6:]
            
            print(f"Response: '{response_text}'")
            return response_text
            
        except Exception as e:
            print(f"Error generating response: {e}")
            return "I'm sorry, I encountered an error processing your request."
    
    async def think_stream(self, user_input: str):
        """
        Process user input and stream the response token by token.
        Uses FAQ → RAG → LLM pipeline for optimal speed.
        
        Args:
            user_input: The user's query or statement
        
        Yields:
            Response text chunks as they are generated
        """
        # Step 1: Try FAQ first (instant, no streaming needed)
        if self.use_faq and self.faq_database:
            faq_answer = self.faq_database.get_answer(user_input)
            if faq_answer:
                print(f"FAQ Hit! Instant response")
                # Update conversation history
                self.conversation_history.append(HumanMessage(content=user_input))
                from langchain_core.messages import AIMessage
                self.conversation_history.append(AIMessage(content=faq_answer))
                
                # Keep history manageable
                if len(self.conversation_history) > 6:
                    self.conversation_history = self.conversation_history[-6:]
                
                yield faq_answer
                return
        
        # Step 2: Check if query needs web search
        web_context = ""
        if self.use_web_search and self.web_search:
            if self.web_search.needs_web_search(user_input):
                # Yield random filler message while searching
                filler_messages = [
                    "Mmm, let me search the internet for that. ",
                    "Let me look that up online for you. ",
                    "Give me a moment, I'll search the web. ",
                    "Let me check the internet real quick. ",
                    "Hold on, searching the web for you. "
                ]
                yield random.choice(filler_messages)
                web_context = self.web_search.search(user_input, max_results=1)  # Reduced from 2 for speed
        
        # Step 3: Check if query needs department knowledge
        needs_rag = self._needs_department_knowledge(user_input)
        
        if not needs_rag and not web_context:
            print(f"Simple query detected - Skipping RAG for faster response")
        
        # Step 4: Build system prompt (with RAG and/or web search)
        system_prompt = self._build_system_prompt(user_input, skip_rag=not needs_rag, web_context=web_context)
        
        # Build messages
        messages = [SystemMessage(content=system_prompt)]
        messages.extend(self.conversation_history)
        messages.append(HumanMessage(content=user_input))
        
        print(f"Thinking about: '{user_input}'")
        
        try:
            full_response = ""
            
            # Stream response chunks
            async for chunk in self.llm.astream(messages):
                if chunk.content:
                    full_response += chunk.content
                    yield chunk.content
            
            # Update conversation history
            self.conversation_history.append(HumanMessage(content=user_input))
            from langchain_core.messages import AIMessage
            self.conversation_history.append(AIMessage(content=full_response))
            
            # Keep history manageable
            if len(self.conversation_history) > 6:
                self.conversation_history = self.conversation_history[-6:]
            
            print(f"Response: '{full_response}'")
            
        except Exception as e:
            print(f"Error generating response: {e}")
            yield "I'm sorry, I encountered an error processing your request."
    
    def clear_history(self):
        """Clear the conversation history."""
        self.conversation_history = []
        print("Conversation history cleared")
    
    def set_system_prompt(self, prompt: str):
        """Update the system prompt."""
        self.system_prompt = prompt
        print("System prompt updated")
