#include <iostream>
#include <thread>
#include "audio_player/audio_player.h"


int main()
{
  // AudioPlayerオブジェクトの作成
  AudioPlayer player;

  // サウンドファイルのロード

  if (!player.loadSound("game_over", "/home/nakao-t/DemoPro2024/src/bullet/sound/game_over_voice.wav"))
  {
    std::cerr << "Failed to load sound file." << std::endl;
    return -1;
  }

  // サウンドの再生
  player.playSound("game_over");

  // サウンドが再生されるのを待つための一時停止
  std::cout << "Playing sound, press Enter to exit..." << std::endl;
  std::cin.get();

  return 0;
}