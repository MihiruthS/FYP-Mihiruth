"""
Web Search Module for Robot Voice Pipeline

Provides internet search capability using Tavily API.
Tavily is designed specifically for LLM applications.
"""

import os
from typing import Optional
from tavily import TavilyClient


class WebSearch:
    """
    Web search interface using Tavily API.
    
    Tavily provides AI-optimized search results perfect for RAG applications.
    """
    
    def __init__(self, api_key: Optional[str] = None):
        """Initialize the Tavily search client."""
        self.api_key = api_key or os.getenv("TAVILY_API_KEY")
        if not self.api_key:
            raise ValueError("TAVILY_API_KEY environment variable is required")
        
        self.client = TavilyClient(api_key=self.api_key)
    
    def search(self, query: str, max_results: int = 3) -> str:
        """
        Search the web and return a summarized context.
        
        Args:
            query: Search query
            max_results: Maximum number of results to return (default: 3)
        
        Returns:
            Formatted search results as a string
        """
        try:
            print(f"Searching web with Tavily: '{query}'")
            
            # Perform search with Tavily
            response = self.client.search(
                query=query,
                max_results=max_results,
                search_depth="basic",  # "basic" or "advanced"
                include_answer=True,   # Get AI-generated answer
                include_raw_content=False
            )
            
            # Check if we have results
            if not response or 'results' not in response:
                print("No web results found")
                return ""
            
            results = response.get('results', [])
            answer = response.get('answer', '')
            
            if not results:
                print("No web results found")
                return ""
            
            # Format results into context
            context_parts = []
            
            # Add Tavily's AI-generated answer if available
            if answer:
                context_parts.append(f"AI SUMMARY: {answer}")
            
            # Add individual search results
            for i, result in enumerate(results[:max_results], 1):
                title = result.get('title', 'No title')
                content = result.get('content', 'No description')
                url = result.get('url', '')
                
                # Format result
                result_text = f"[Source {i}] {title}\n{content}"
                if url:
                    result_text += f"\n(Source: {url})"
                
                context_parts.append(result_text)
            
            context = "WEB SEARCH RESULTS:\n" + "\n---\n".join(context_parts)
            print(f"Found {len(results)} web results")
            
            # Debug: show summary
            if answer:
                print(f"Tavily summary: {answer[:80]}...")
            
            return context
            
        except Exception as e:
            print(f"Web search failed: {e}")
            return ""
    
    def needs_web_search(self, query: str) -> bool:
        """
        Determine if a query needs web search.
        
        PRIORITY: Department questions ALWAYS use RAG, never web search.
        Web search is ONLY for external/world knowledge.
        
        Args:
            query: User query
        
        Returns:
            True if web search should be used
        """
        query_lower = query.lower()
        
        # PRIORITY 1: Department-specific patterns EXCLUDE from web search
        # These should ALWAYS use RAG, never web search
        department_exclusions = [
            "department", "entc", "moratuwa", "university of moratuwa",
            "lab", "laboratory", "lecturer", "professor", "dr ",
            "head of department", "hod", "computer lab",
            "office", "building", "room", "floor", "campus",
            "electronic club", "electronics club", "e-club",
            "student club", "student association",
            "biomedical", "telecommunication", "engineering department"
        ]
        
        # If it contains ANY department keywords, DON'T use web search
        if any(excl in query_lower for excl in department_exclusions):
            return False
        
        # PRIORITY 2: Skip web search for time/date (available in system prompt)
        time_date_patterns = [
            "what is the time", "what's the time", "current time",
            "what is the date", "what's the date", "today's date",
            "what time is it", "what date is it", "tell me the time",
            "tell me the date"
        ]
        if any(pattern in query_lower for pattern in time_date_patterns):
            return False
        
        # PRIORITY 3: Patterns that indicate need for EXTERNAL web search
        web_patterns = [
            # World leaders & politics (specific countries)
            "president of usa", "president of america", "president of united states",
            "president of maldives", "president of sri lanka", "president of india",
            "president of china", "president of russia", "president of france",
            "prime minister of", "government of", "election in",
            
            # General world knowledge
            "most powerful nation", "strongest country", "richest country",
            "largest country", "smallest country", "most populated",
            
            # Geography & demographics
            "capital of", "population of", "currency of",
            
            # Sports
            "captain of", "champion", "winner", "world cup", "olympics",
            "formula one", "f1", "cricket team", "football team", "soccer",
            "tournament", "match result", "game score",
            
            # Entertainment
            "movie", "film", "actor", "actress", "singer",
            "album", "song", "tv show", "series",
            
            # Current events & news
            "latest news", "breaking news", "current events", "recent news",
            
            # Weather
            "weather in", "temperature in", "climate in"
        ]
        
        # If it matches web patterns, use web search
        if any(pattern in query_lower for pattern in web_patterns):
            return True
        
        return False
