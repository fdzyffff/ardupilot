#include "Plane.h"

void Plane::hxky_init()
{
    AP::hxky_weight().init();
    AP::hxky_engines().init();
}

void Plane::hxky_update_100hz()
{
    AP::hxky_weight().update();
    AP::hxky_engines().update();
    uart.update();
}

void Plane::hxky_update_1hz()
{
    AP::hxky_weight().do_print();
}

void Plane::handle_msg_hxky(const mavlink_message_t &msg)
{
    AP::hxky_engines().handle_message(msg);
    AP::hxky_weight().handle_message(msg);
}
