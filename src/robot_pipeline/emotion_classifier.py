"""
Emotion Classifier for Robot Pipeline

Analyzes text to classify emotions without adding latency to speech pipeline.
Runs asynchronously in the background while robot speaks.
"""

import asyncio
from typing import Literal
import openai
import os

EmotionType = Literal["neutral", "joy", "fear", "disgust", "sadness", "anger", "surprise"]


class EmotionClassifier:
    """
    Classifies text into one of 7 emotions: neutral, joy, fear, disgust, sadness, anger, surprise.
    
    Uses OpenAI API for accurate emotion classification with minimal latency.
    Designed to run asynchronously without blocking the speaking pipeline.
    """
    
    def __init__(self):
        """Initialize the emotion classifier."""
        self.client = openai.AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        
        # System prompt for emotion classification
        self.system_prompt = """You are an emotion classifier for a robot's facial expressions.
Analyze the given text and classify it into ONE of these emotions:
- neutral: Informational, factual, calm responses
- joy: Happy, positive, excited, enthusiastic responses
- fear: Worried, anxious, uncertain responses
- disgust: Disapproval, dislike, unpleasant topics
- sadness: Sad, disappointed, sympathetic, apologetic responses
- anger: Frustrated, annoyed, stern responses
- surprise: Unexpected information, amazement, shock

Respond with ONLY the emotion name, nothing else."""

        # Cache for recent classifications to avoid redundant API calls
        self._cache = {}
        self._cache_size = 50
    
    async def classify(self, text: str) -> EmotionType:
        """
        Classify text into an emotion asynchronously.
        
        Args:
            text: The text to classify
            
        Returns:
            One of: "neutral", "joy", "fear", "disgust", "sadness", "anger", "surprise"
        """
        if not text or not text.strip():
            return "neutral"
        
        # Check cache first
        cache_key = text.lower().strip()[:100]  # Use first 100 chars as key
        if cache_key in self._cache:
            return self._cache[cache_key]
        
        try:
            # Call OpenAI API asynchronously
            response = await self.client.chat.completions.create(
                model="gpt-3.5-turbo",  # Fast and cheap model
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": f"Classify this text:\n\n{text[:500]}"}  # Limit to 500 chars
                ],
                temperature=0.3,  # Low temperature for consistent classification
                max_tokens=10,  # We only need one word
                timeout=3.0  # 3 second timeout
            )
            
            emotion = response.choices[0].message.content.strip().lower()
            
            # Validate emotion
            valid_emotions = ["neutral", "joy", "fear", "disgust", "sadness", "anger", "surprise"]
            if emotion not in valid_emotions:
                print(f"⚠️  Invalid emotion '{emotion}', defaulting to neutral")
                emotion = "neutral"
            
            # Cache result
            self._cache[cache_key] = emotion
            if len(self._cache) > self._cache_size:
                # Remove oldest entry
                self._cache.pop(next(iter(self._cache)))
            
            return emotion
            
        except asyncio.TimeoutError:
            print("⚠️  Emotion classification timed out, using neutral")
            return "neutral"
        except Exception as e:
            print(f"⚠️  Emotion classification error: {e}, using neutral")
            return "neutral"
    
    async def classify_streaming(self, text_stream) -> EmotionType:
        """
        Classify emotion from a stream of text chunks.
        
        Waits until enough text is accumulated (min 20 chars or end of stream)
        before classifying to get better accuracy.
        
        Args:
            text_stream: Async generator yielding text chunks
            
        Returns:
            Classified emotion
        """
        accumulated_text = ""
        min_chars = 20  # Wait for at least 20 chars before classifying
        
        async for chunk in text_stream:
            accumulated_text += chunk
            
            # If we have enough text, classify it
            if len(accumulated_text) >= min_chars:
                emotion = await self.classify(accumulated_text)
                return emotion
        
        # Classify whatever we got
        if accumulated_text:
            return await self.classify(accumulated_text)
        
        return "neutral"
    
    def classify_simple(self, text: str) -> EmotionType:
        """
        Simple rule-based emotion classification (fast, no API call).
        
        Use this as a fallback when API is unavailable or for very quick classification.
        
        Args:
            text: The text to classify
            
        Returns:
            One of: "neutral", "joy", "fear", "disgust", "sadness", "anger", "surprise"
        """
        text_lower = text.lower()
        
        # Joy keywords
        joy_words = ["happy", "great", "excellent", "wonderful", "amazing", "fantastic", 
                     "thank", "thanks", "glad", "love", "excited", "perfect", "awesome"]
        if any(word in text_lower for word in joy_words):
            return "joy"
        
        # Sadness keywords
        sad_words = ["sorry", "apolog", "unfortunately", "sad", "disappoint", "regret"]
        if any(word in text_lower for word in sad_words):
            return "sadness"
        
        # Fear keywords
        fear_words = ["worried", "concern", "afraid", "nervous", "unsure", "uncertain"]
        if any(word in text_lower for word in fear_words):
            return "fear"
        
        # Surprise keywords
        surprise_words = ["wow", "really", "seriously", "no way", "surprising", "unexpected", "?!"]
        if any(word in text_lower for word in surprise_words):
            return "surprise"
        
        # Anger keywords
        anger_words = ["angry", "furious", "unacceptable", "ridiculous", "terrible", "awful"]
        if any(word in text_lower for word in anger_words):
            return "anger"
        
        # Default to neutral
        return "neutral"
