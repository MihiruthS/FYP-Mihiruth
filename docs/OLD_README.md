# Robot Voice Pipeline 🤖🎤

A complete voice interaction pipeline for robots built with Python. The pipeline listens to user queries, processes them with AI, and responds with natural-sounding speech.

## Features

- **🎤 Voice Input**: Real-time microphone audio capture
- **📝 Speech-to-Text**: AssemblyAI streaming transcription
- **🤔 AI Processing**: OpenAI GPT-powered conversational agent
- **🗣️ Text-to-Speech**: Cartesia Sonic high-quality voice synthesis
- **🔊 Audio Output**: Real-time audio playback
- **🔄 Continuous Loop**: Automatically listens for next query after responding

## Pipeline Flow

```
Listen (Microphone) 
    ↓
Transcribe (AssemblyAI STT)
    ↓
Think (OpenAI GPT)
    ↓
Synthesize (Cartesia TTS)
    ↓
Speak (Audio Playback)
    ↓
Loop back to Listen
```

## Installation

### Prerequisites

- Python 3.11 or higher
- Microphone and speakers/audio output
- API keys for:
  - [AssemblyAI](https://www.assemblyai.com/) (Speech-to-Text)
  - [Cartesia](https://cartesia.ai/) (Text-to-Speech)
  - [OpenAI](https://platform.openai.com/) (AI Agent)

### Install Dependencies

```bash
# Install PyAudio (may require system dependencies)
# On Ubuntu/Debian:
sudo apt-get install portaudio19-dev python3-pyaudio

# On macOS:
brew install portaudio

# On Windows:
# PyAudio wheels available at: https://www.lfd.uci.edu/~gohlke/pythonlibs/#pyaudio

# Install Python dependencies
pip install pyaudio websockets langchain langchain-openai langchain-core python-dotenv
```

### Configuration

1. Copy the environment template:
```bash
cp .env.example .env
```

2. Edit `.env` and add your API keys:
```bash
ASSEMBLYAI_API_KEY=your_assemblyai_api_key_here
CARTESIA_API_KEY=your_cartesia_api_key_here
OPENAI_API_KEY=your_openai_api_key_here
```

## Usage

### Run the Pipeline

```bash
cd src
python main.py
```

### How It Works

1. **Start**: The pipeline initializes and waits for audio input
2. **Listen**: Speak your query within 5 seconds (configurable)
3. **Process**: The robot transcribes, thinks, and responds
4. **Repeat**: Automatically returns to listening mode

### Exit Commands

Say one of the following to exit:
- "exit"
- "quit"
- "stop"
- "goodbye"
- "bye"

Or press `Ctrl+C` to force quit.

## Project Structure

```
robot-voice-pipeline/
├── src/
│   ├── main.py                # Main pipeline orchestrator
│   ├── audio_capture.py       # Microphone input handling
│   ├── audio_playback.py      # Speaker output handling
│   ├── speech_to_text.py      # AssemblyAI STT integration
│   ├── text_to_speech.py      # Cartesia TTS integration
│   └── ai_agent.py            # OpenAI LangChain agent
├── pyproject.toml             # Python project configuration
├── .env.example               # Environment variables template
└── README.md                  # This file
```

## Module Details

### audio_capture.py
- Captures audio from microphone using PyAudio
- Provides async stream of audio chunks
- Format: 16kHz, mono, 16-bit PCM

### speech_to_text.py
- Connects to AssemblyAI's streaming API
- Real-time speech transcription
- Returns complete formatted transcripts

### ai_agent.py
- LangChain-powered conversational agent
- Uses OpenAI GPT models
- Maintains conversation history
- Customizable system prompts

### text_to_speech.py
- Cartesia Sonic TTS integration
- High-quality natural voice synthesis
- Supports multiple voices and languages
- Output: 24kHz, mono, 16-bit PCM

### audio_playback.py
- Plays audio through system speakers
- Async playback support
- Handles PCM audio format

### main.py
- Orchestrates the complete pipeline
- Manages component lifecycle
- Handles errors and cleanup
- Configurable listen duration

## Customization

### Change Listen Duration

Edit [main.py](src/main.py):
```python
pipeline = RobotVoicePipeline(
    listen_duration=10.0,  # Listen for 10 seconds
)
```

### Customize AI Agent Personality

Edit [main.py](src/main.py):
```python
pipeline = RobotVoicePipeline(
    system_prompt="You are a friendly robot companion who loves jokes..."
)
```

### Change Voice

Edit [text_to_speech.py](src/text_to_speech.py) or pass to TextToSpeech:
```python
tts = TextToSpeech(
    voice_id="different-voice-id",  # See Cartesia docs for voice IDs
)
```

### Change AI Model

Edit [ai_agent.py](src/ai_agent.py):
```python
agent = AIAgent(
    model="gpt-4",  # Use GPT-4 instead of gpt-4o-mini
)
```

## Troubleshooting

### Audio Issues

**Microphone not detected:**
- Check system audio settings
- Verify PyAudio installation
- Test with: `python -c "import pyaudio; p = pyaudio.PyAudio(); print(p.get_default_input_device_info())"`

**No audio output:**
- Check speaker volume
- Verify audio playback device
- Test with: `python -c "import pyaudio; p = pyaudio.PyAudio(); print(p.get_default_output_device_info())"`

### API Issues

**Authentication errors:**
- Verify API keys in `.env` file
- Check API key validity and quotas
- Ensure `.env` is in the correct location

**Connection errors:**
- Check internet connection
- Verify firewall settings
- Check API service status

## Performance Notes

- **Latency**: ~1-3 seconds from speech end to response start
- **Listen Duration**: Default 5 seconds (configurable)
- **Memory**: Maintains last 10 conversation turns
- **Audio Quality**: 
  - Input: 16kHz (STT optimized)
  - Output: 24kHz (High quality TTS)

## License

MIT License - Feel free to use and modify for your robot projects!

## Credits

Built with:
- [AssemblyAI](https://www.assemblyai.com/) - Speech-to-Text
- [Cartesia](https://cartesia.ai/) - Text-to-Speech
- [OpenAI](https://openai.com/) - Language Model
- [LangChain](https://langchain.com/) - LLM Framework
- [PyAudio](https://people.csail.mit.edu/hubert/pyaudio/) - Audio I/O

## Support

For issues or questions:
1. Check the Troubleshooting section
2. Review API provider documentation
3. Check system audio configuration

---

**Happy Robot Building! 🤖✨**
