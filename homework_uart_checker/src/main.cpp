#include <unistd.h>
#include <drone_link.h>

int openUart(const char* dev);

int main(int argc, char** argv)
{
    auto fd = openUart("/dev/ttyAMA3");

    dlink::Parser parser; // тримає стан між викликами
    uint8_t buf[256];
    while (true) {
        int n = read(fd, buf, sizeof(buf)); // прочитати доступні байти
        uint8_t type, len, payload[260];
        for (int i = 0; i < n; i++) {
            if (parser.feed(buf[i], type, payload, len)) { // зібрався цілий кадр
                if (type == dlink::PKT_TELEMETRY) {
                    dlink::Telemetry telemetry;
                    memcpy(&telemetry, payload, sizeof telemetry);
                    // отримані дані телеметрії
                }
                if (type == dlink::PKT_TARGET) {
                    dlink::TargetPos targetPos;
                    memcpy(&targetPos, payload, sizeof targetPos);
                    // отримані дані таргету
                }
                if (type == dlink::PKT_AMMO) {
                    dlink::AmmoCfg ammoCfg;
                    memcpy(&ammoCfg, payload, sizeof ammoCfg);
                    // отримані дані снаряду
                }
            }
        }
    }
}