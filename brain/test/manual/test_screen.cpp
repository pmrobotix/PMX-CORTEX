/*!
 * \file test_screen.cpp
 * \brief Test manuel de l'ecran TFT 7" (framebuffer) et du tactile resistif.
 *
 * Ecran : Santek ST0700-Adapt (800x480, 18 bits, tactile resistif 4 fils)
 * Framebuffer : /dev/fb0 (mxsfb-drm)
 * Touchscreen : /dev/input/event0 (TSC i.MX6ULL)
 *
 * Usage : ./test-screen [--touch]
 *   Sans argument : test framebuffer uniquement (couleurs + texte info)
 *   --touch       : test framebuffer + lecture tactile
 */

#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

// --- Framebuffer ---

struct Framebuffer {
    int fd = -1;
    uint8_t* mem = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t bpp = 0;       // bits per pixel
    uint32_t lineLen = 0;   // bytes per line
    size_t size = 0;

    bool open(const char* device = "/dev/fb0")
    {
        fd = ::open(device, O_RDWR);
        if (fd < 0) {
            std::cerr << "Erreur ouverture " << device << std::endl;
            return false;
        }

        fb_var_screeninfo vinfo{};
        fb_fix_screeninfo finfo{};
        ioctl(fd, FBIOGET_VSCREENINFO, &vinfo);
        ioctl(fd, FBIOGET_FSCREENINFO, &finfo);

        width = vinfo.xres;
        height = vinfo.yres;
        bpp = vinfo.bits_per_pixel;
        lineLen = finfo.line_length;
        size = lineLen * height;

        mem = static_cast<uint8_t*>(mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
        if (mem == MAP_FAILED) {
            std::cerr << "Erreur mmap framebuffer" << std::endl;
            ::close(fd);
            fd = -1;
            return false;
        }

        std::cout << "Framebuffer : " << width << "x" << height
                  << " " << bpp << "bpp, line=" << lineLen << " bytes" << std::endl;
        return true;
    }

    void close()
    {
        if (mem && mem != MAP_FAILED) munmap(mem, size);
        if (fd >= 0) ::close(fd);
        fd = -1;
        mem = nullptr;
    }

    // Pixel 32bpp (ARGB) ou 16bpp (RGB565)
    void setPixel(uint32_t x, uint32_t y, uint8_t r, uint8_t g, uint8_t b)
    {
        if (x >= width || y >= height) return;
        uint32_t offset = y * lineLen + x * (bpp / 8);

        if (bpp == 32) {
            mem[offset + 0] = b;
            mem[offset + 1] = g;
            mem[offset + 2] = r;
            mem[offset + 3] = 0xFF;
        } else if (bpp == 16) {
            uint16_t pixel = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
            *reinterpret_cast<uint16_t*>(mem + offset) = pixel;
        }
    }

    void fillRect(uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                  uint8_t r, uint8_t g, uint8_t b)
    {
        for (uint32_t py = y; py < y + h && py < height; ++py)
            for (uint32_t px = x; px < x + w && px < width; ++px)
                setPixel(px, py, r, g, b);
    }

    void fillScreen(uint8_t r, uint8_t g, uint8_t b)
    {
        fillRect(0, 0, width, height, r, g, b);
    }

    // Dessine un petit carre a une position (pour marquer un touch)
    void drawCross(uint32_t cx, uint32_t cy, uint8_t r, uint8_t g, uint8_t b)
    {
        for (int d = -5; d <= 5; ++d) {
            setPixel(cx + d, cy, r, g, b);
            setPixel(cx, cy + d, r, g, b);
        }
    }
};

// --- Test framebuffer : affiche des bandes de couleur ---

void testFramebuffer(Framebuffer& fb)
{
    std::cout << "\n--- Test Framebuffer ---" << std::endl;

    // Bande rouge
    std::cout << "Rouge..." << std::endl;
    fb.fillScreen(255, 0, 0);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Bande verte
    std::cout << "Vert..." << std::endl;
    fb.fillScreen(0, 255, 0);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Bande bleue
    std::cout << "Bleu..." << std::endl;
    fb.fillScreen(0, 0, 255);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Blanc
    std::cout << "Blanc..." << std::endl;
    fb.fillScreen(255, 255, 255);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Bandes verticales : 8 couleurs
    std::cout << "Mire 8 couleurs..." << std::endl;
    fb.fillScreen(0, 0, 0);
    uint32_t bandW = fb.width / 8;
    uint8_t colors[][3] = {
        {255,255,255}, {255,255,0}, {0,255,255}, {0,255,0},
        {255,0,255}, {255,0,0}, {0,0,255}, {0,0,0}
    };
    for (int i = 0; i < 8; ++i) {
        fb.fillRect(i * bandW, 0, bandW, fb.height,
                    colors[i][0], colors[i][1], colors[i][2]);
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Ecran noir avec info
    fb.fillScreen(0, 0, 0);
    std::cout << "Test framebuffer OK." << std::endl;
}

// --- Test tactile : lit les events et dessine des croix ---

void testTouchscreen(Framebuffer& fb, const char* evDevice = "/dev/input/event0")
{
    std::cout << "\n--- Test Tactile ---" << std::endl;
    std::cout << "Toucher l'ecran pour dessiner (Ctrl+C pour quitter)" << std::endl;

    // Fond bleu fonce
    fb.fillScreen(0, 0, 64);

    int evFd = ::open(evDevice, O_RDONLY);
    if (evFd < 0) {
        std::cerr << "Erreur ouverture " << evDevice << std::endl;
        std::cerr << "Verifier que le device existe et les permissions (root ?)" << std::endl;
        return;
    }

    // Lire les plages de l'ecran tactile
    struct input_absinfo absX{}, absY{};
    ioctl(evFd, EVIOCGABS(ABS_X), &absX);
    ioctl(evFd, EVIOCGABS(ABS_Y), &absY);
    std::cout << "Touch X range : " << absX.minimum << " - " << absX.maximum << std::endl;
    std::cout << "Touch Y range : " << absY.minimum << " - " << absY.maximum << std::endl;

    int touchX = 0, touchY = 0;
    bool pressed = false;

    struct input_event ev{};
    while (read(evFd, &ev, sizeof(ev)) == sizeof(ev)) {
        if (ev.type == EV_ABS) {
            if (ev.code == ABS_X) {
                // Mapper la valeur brute vers les pixels ecran
                touchX = (ev.value - absX.minimum) * static_cast<int>(fb.width)
                         / (absX.maximum - absX.minimum);
            } else if (ev.code == ABS_Y) {
                touchY = (ev.value - absY.minimum) * static_cast<int>(fb.height)
                         / (absY.maximum - absY.minimum);
            }
        } else if (ev.type == EV_KEY && ev.code == BTN_TOUCH) {
            pressed = (ev.value != 0);
            if (pressed) {
                std::cout << "Touch @ " << touchX << "," << touchY << std::endl;
            }
        } else if (ev.type == EV_SYN && pressed) {
            // Dessiner une croix jaune a la position
            fb.drawCross(touchX, touchY, 255, 255, 0);
        }
    }

    ::close(evFd);
}

// --- Main ---

int main(int argc, char** argv)
{
    std::cout << "=== PMX-CORTEX : Test Ecran TFT 7\" ===" << std::endl;

    bool doTouch = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--touch")
            doTouch = true;
    }

    Framebuffer fb;
    if (!fb.open("/dev/fb0")) {
        return 1;
    }

    testFramebuffer(fb);

    if (doTouch) {
        testTouchscreen(fb);
    } else {
        std::cout << "\nRelancer avec --touch pour tester le tactile." << std::endl;
    }

    fb.close();
    std::cout << "=== Fin ===" << std::endl;
    return 0;
}
