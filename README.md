# Using Waveshare 1.3" LCD Module for Raspberry Pi Pico with Rust

[![](https://img.youtube.com/vi/UnVssEl5PBc/0.jpg)](https://www.youtube.com/watch?v=UnVssEl5PBc)

This project is for controlling an [LCD module(using ST7789)](https://www.waveshare.com/wiki/Pico-LCD-1.3) for a Raspberry Pi Pico using Rust.  
The code is primitive, so not much can be done with it, but we were able to actually control the screen using Rust.  

## Blog

[Rust初心者が、Rust+RPi PicoでST7789使用TFT LCD(Waveshare Pico LCD 1.3)を使う話(In Japanese)](https://www.shtsno24.tokyo/2022/05/rustrustrpi-picost7789tft-lcdwaveshare.html)

## Reference

* [Code Template](https://github.com/rp-rs/rp2040-project-template)   
* [Building a Development Environment(in Japanese)](https://qiita.com/ochaochaocha3/items/1969d76debd6d3b42269)
* [SPI](https://github.com/rp-rs/rp-hal/blob/main/rp2040-hal/examples/spi.rs)
* [GPIO Wrapper](https://github.com/rp-rs/rp-hal/blob/main/boards/rp-pico/examples/pico_spi_sd_card.rs)
