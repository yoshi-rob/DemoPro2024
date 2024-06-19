#include <iostream>

#include "audio_player/audio_player.h"

bool AudioPlayer::loadSound(const std::string &name, const std::string &file_path)
{
  sf::SoundBuffer buffer;
  if (!buffer.loadFromFile(file_path))
  {
    std::cerr << "Error loading " << file_path << std::endl;
    return false;
  }
  buffers[name] = buffer;

  sf::Sound sound;
  sound.setBuffer(buffers[name]);
  sounds[name] = sound;

  return true;
}

void AudioPlayer::playSound(const std::string &name)
{
  if (sounds.find(name) != sounds.end())
  {
    sounds[name].play();
  }
  else
  {
    std::cerr << "Sound " << name << " not found!" << std::endl;
  }
}

