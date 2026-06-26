#include "Plane.h"

void Plane::hxky_weight_update()
{
    AP::hxky_weight().update();
}

void Plane::hxky_engine_update()
{
    AP::hxky_engines().update();
}

void Plane::hxky_uart_update()
{
    AP::hxky_uart().update();
    uart.update();  // 业务层处理
}

void Plane::hxky_one_hz()
{
    AP::hxky_weight().do_print();
}

void GCS_MAVLINK_Plane::handle_msg_hxky(const mavlink_message_t &msg)
{
    AP::hxky_engines().handle_message(msg);
    AP::hxky_weight().handle_message(msg);
}
