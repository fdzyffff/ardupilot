#include "Copter.h"

void Copter::hxky_init()
{
    AP::hxky_uart().init();
}

void Copter::hxky_weight_update()
{
    AP::hxky_weight().update();
}

void Copter::hxky_engine_update()
{
    AP::hxky_engines().update();
}

void Copter::hxky_uart_update()
{
    AP::hxky_uart().update();
}

void Copter::hxky_one_hz()
{
    AP::hxky_weight().do_print();
}

void GCS_MAVLINK_Copter::handle_msg_hxky(const mavlink_message_t &msg)
{
    AP::hxky_engines().handle_message(msg);
    AP::hxky_weight().handle_message(msg);
}
