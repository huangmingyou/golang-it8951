# golang-it8951

微雪10.3寸墨水屏配置的it8915 display HAT, 官方提供了gpio版本的demo。同时还提供了windows版本的usb驱动。但是没有提供linux版本的usb驱动。

https://git.sr.ht/~martijnbraam/it8951 项目，通过对官方的windows usb demo 进行usb抓包，模拟和it8951的scsi信令。实现了linux下对it8951的usb驱动。

本项目通过golang对这个c进行扩展，可以通过http接口上传图片，然后展示在墨水屏上。方便二次开发。

