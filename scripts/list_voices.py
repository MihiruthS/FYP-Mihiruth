"""
Utility script to list all available Cartesia TTS voices.

Run this script to see all voice options with their IDs, names, and descriptions.
"""

import os
import json
import urllib.request

# Try to load from .env file
try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass  # dotenv not installed, use environment variables directly


def list_voices():
    """Fetch and display all available Cartesia voices."""
    api_key = os.getenv("CARTESIA_API_KEY")
    if not api_key:
        print("CARTESIA_API_KEY not found in environment variables")
        return
    
    url = "https://api.cartesia.ai/voices"
    
    print("Fetching available Cartesia voices...\n")
    
    try:
        req = urllib.request.Request(url)
        req.add_header("X-API-Key", api_key)
        req.add_header("Cartesia-Version", "2024-06-10")
        
        with urllib.request.urlopen(req) as response:
            if response.status == 200:
                voices = json.loads(response.read().decode('utf-8'))
                
                print(f"Found {len(voices)} voices:\n")
                print("="*80)
                
                for voice in voices:
                    voice_id = voice.get("id", "N/A")
                    name = voice.get("name", "N/A")
                    language = voice.get("language", "N/A")
                    description = voice.get("description", "")
                    
                    print(f"Voice ID: {voice_id}")
                    print(f"Name: {name}")
                    print(f"Language: {language}")
                    if description:
                        print(f"Description: {description}")
                    print("-"*80)
                    
                # Highlight current voice being used
                print("\nCURRENTLY USED IN PIPELINE:")
                print("Voice ID: f6ff7c0c-e396-40a9-a70b-f7607edb6937")
                current_voice = next(
                    (v for v in voices if v.get("id") == "f6ff7c0c-e396-40a9-a70b-f7607edb6937"), 
                    None
                )
                if current_voice:
                    print(f"Name: {current_voice.get('name', 'N/A')}")
                    print(f"Description: {current_voice.get('description', 'N/A')}")
                
            else:
                print(f"Error: API returned status code {response.status}")
                
    except Exception as e:
        print(f"Error fetching voices: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    list_voices()
