/*!
 * \file
 * \brief Test manuel de l'ecran TFT 7" (framebuffer) et du tactile resistif.
 */

#include "ScreenFramebufferManualTest.hpp"

#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <fstream>
#include <sstream>

static const char* CALIBRATION_FILE = "touch-calibration.conf";

// --- Calibration persistence ---

struct TouchCalibration {
	double scaleX = 0.0;
	double offsetX = 0.0;
	double scaleY = 0.0;
	double offsetY = 0.0;
	bool valid = false;

	bool load()
	{
		std::ifstream f(CALIBRATION_FILE);
		if (!f.is_open()) return false;

		std::string line;
		int count = 0;
		while (std::getline(f, line)) {
			if (line.empty() || line[0] == '#') continue;
			std::istringstream iss(line);
			std::string key;
			double val;
			if (iss >> key >> val) {
				if (key == "scaleX") { scaleX = val; count++; }
				else if (key == "offsetX") { offsetX = val; count++; }
				else if (key == "scaleY") { scaleY = val; count++; }
				else if (key == "offsetY") { offsetY = val; count++; }
			}
		}
		valid = (count == 4);
		return valid;
	}

	bool save() const
	{
		std::ofstream f(CALIBRATION_FILE);
		if (!f.is_open()) {
			std::cerr << "Impossible d'ecrire " << CALIBRATION_FILE << std::endl;
			return false;
		}
		f << "# Calibration tactile (generee automatiquement)\n";
		f << "scaleX " << scaleX << "\n";
		f << "offsetX " << offsetX << "\n";
		f << "scaleY " << scaleY << "\n";
		f << "offsetY " << offsetY << "\n";
		std::cout << "Calibration sauvegardee dans " << CALIBRATION_FILE << std::endl;
		return true;
	}
};

// --- Framebuffer helper ---

struct Framebuffer {
	int fd = -1;
	uint8_t* mem = nullptr;
	uint32_t width = 0;
	uint32_t height = 0;
	uint32_t bpp = 0;
	uint32_t lineLen = 0;
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

	void drawCross(uint32_t cx, uint32_t cy, uint32_t size, uint8_t r, uint8_t g, uint8_t b)
	{
		for (int d = -static_cast<int>(size); d <= static_cast<int>(size); ++d) {
			setPixel(cx + d, cy, r, g, b);
			setPixel(cx, cy + d, r, g, b);
		}
	}
};

// --- Tests ---

void test::ScreenFramebufferManualTest::suite()
{
	testVerifyCalibration();
	testFramebuffer();
	testTouchscreen();
	testCalibration();
}

void test::ScreenFramebufferManualTest::testFramebuffer()
{
	Framebuffer fb;
	if (!fb.open("/dev/fb0")) {
		this->fail("Impossible d'ouvrir /dev/fb0");
		return;
	}

	std::cout << "\n--- Test Framebuffer ---" << std::endl;

	std::cout << "Rouge..." << std::endl;
	fb.fillScreen(255, 0, 0);
	std::this_thread::sleep_for(std::chrono::seconds(1));

	std::cout << "Vert..." << std::endl;
	fb.fillScreen(0, 255, 0);
	std::this_thread::sleep_for(std::chrono::seconds(1));

	std::cout << "Bleu..." << std::endl;
	fb.fillScreen(0, 0, 255);
	std::this_thread::sleep_for(std::chrono::seconds(1));

	std::cout << "Blanc..." << std::endl;
	fb.fillScreen(255, 255, 255);
	std::this_thread::sleep_for(std::chrono::seconds(1));

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

	fb.fillScreen(0, 0, 0);
	fb.close();

	this->assert(true, "Test framebuffer OK");
}

void test::ScreenFramebufferManualTest::testTouchscreen()
{
	const char* evDevice = "/dev/input/event0";

	int evFd = ::open(evDevice, O_RDONLY | O_NONBLOCK);
	if (evFd < 0) {
		std::cerr << "Pas de device tactile " << evDevice
				  << " (skip test touch)" << std::endl;
		this->assert(true, "Test touch skipped (pas de device)");
		return;
	}

	Framebuffer fb;
	if (!fb.open("/dev/fb0")) {
		::close(evFd);
		this->fail("Impossible d'ouvrir /dev/fb0 pour test touch");
		return;
	}

	std::cout << "\n--- Test Tactile (5 secondes) ---" << std::endl;

	// Charger la calibration si disponible
	TouchCalibration cal;
	bool useCalibration = cal.load();
	if (useCalibration) {
		std::cout << "Calibration chargee depuis " << CALIBRATION_FILE << std::endl;
	} else {
		std::cout << "Pas de calibration, utilisation du mapping lineaire" << std::endl;
	}

	std::cout << "Toucher l'ecran pour dessiner..." << std::endl;

	fb.fillScreen(0, 0, 64);

	struct input_absinfo absX{}, absY{};
	ioctl(evFd, EVIOCGABS(ABS_X), &absX);
	ioctl(evFd, EVIOCGABS(ABS_Y), &absY);
	std::cout << "Touch X range : " << absX.minimum << " - " << absX.maximum << std::endl;
	std::cout << "Touch Y range : " << absY.minimum << " - " << absY.maximum << std::endl;

	int touchX = 0, touchY = 0;
	int rawX = 0, rawY = 0;
	bool pressed = false;
	int touchCount = 0;

	auto start = std::chrono::steady_clock::now();
	struct input_event ev{};

	// Lecture non-bloquante pendant 5 secondes
	while (std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
		ssize_t n = read(evFd, &ev, sizeof(ev));
		if (n != sizeof(ev)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}

		if (ev.type == EV_ABS) {
			if (ev.code == ABS_X) rawX = ev.value;
			else if (ev.code == ABS_Y) rawY = ev.value;

			if (useCalibration) {
				touchX = static_cast<int>(rawX * cal.scaleX + cal.offsetX);
				touchY = static_cast<int>(rawY * cal.scaleY + cal.offsetY);
			} else {
				touchX = (rawX - absX.minimum) * static_cast<int>(fb.width)
						 / (absX.maximum - absX.minimum);
				// Y inversé sur cet ecran resistif
				touchY = static_cast<int>(fb.height)
						 - (rawY - absY.minimum) * static_cast<int>(fb.height)
						 / (absY.maximum - absY.minimum);
			}
		} else if (ev.type == EV_KEY && ev.code == BTN_TOUCH) {
			pressed = (ev.value != 0);
			if (pressed) {
				std::cout << "Touch @ " << touchX << "," << touchY << std::endl;
				touchCount++;
			}
		} else if (ev.type == EV_SYN && pressed) {
			fb.drawCross(touchX, touchY, 5, 255, 255, 0);
		}
	}

	fb.fillScreen(0, 0, 0);
	fb.close();
	::close(evFd);

	std::cout << "Touches detectees : " << touchCount << std::endl;
	this->assert(true, "Test touchscreen OK");
}

void test::ScreenFramebufferManualTest::testVerifyCalibration()
{
	TouchCalibration cal;
	if (!cal.load()) {
		std::cout << "\n--- Verification calibration : SKIP (pas de fichier " << CALIBRATION_FILE << ") ---" << std::endl;
		this->assert(true, "Verification calibration skipped (pas de fichier)");
		return;
	}

	const char* evDevice = "/dev/input/event0";
	int evFd = ::open(evDevice, O_RDONLY | O_NONBLOCK);
	if (evFd < 0) {
		this->assert(true, "Verification calibration skipped (pas de device tactile)");
		return;
	}

	Framebuffer fb;
	if (!fb.open("/dev/fb0")) {
		::close(evFd);
		this->fail("Impossible d'ouvrir /dev/fb0 pour verification");
		return;
	}

	std::cout << "\n--- Verification Calibration (5 points) ---" << std::endl;
	std::cout << "Calibration chargee : scaleX=" << cal.scaleX << " offsetX=" << cal.offsetX
			  << " scaleY=" << cal.scaleY << " offsetY=" << cal.offsetY << std::endl;

	// 5 points de verification : 4 coins + centre
	const uint32_t margin = 50;
	struct VerifyPoint {
		uint32_t targetX;
		uint32_t targetY;
		const char* name;
	};
	VerifyPoint targets[5] = {
		{ margin,              margin,              "haut-gauche" },
		{ fb.width - margin,   margin,              "haut-droite" },
		{ fb.width - margin,   fb.height - margin,  "bas-droite"  },
		{ margin,              fb.height - margin,   "bas-gauche"  },
		{ fb.width / 2,        fb.height / 2,        "centre"      }
	};

	double totalErr = 0.0;
	double maxErr = 0.0;
	int pointsOk = 0;

	for (int i = 0; i < 5; ++i) {
		fb.fillScreen(0, 0, 0);
		fb.drawCross(targets[i].targetX, targets[i].targetY, 10, 255, 255, 255);

		std::cout << "Toucher " << targets[i].name << " (" << targets[i].targetX
				  << "," << targets[i].targetY << ") ..." << std::endl;

		// Purger les evenements en attente
		struct input_event ev{};
		while (read(evFd, &ev, sizeof(ev)) == sizeof(ev)) {}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		// Attendre appui
		bool pressed = false;
		int rawX = 0, rawY = 0;
		auto start = std::chrono::steady_clock::now();

		while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
			ssize_t n = read(evFd, &ev, sizeof(ev));
			if (n != sizeof(ev)) {
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				continue;
			}
			if (ev.type == EV_KEY && ev.code == BTN_TOUCH && ev.value == 1) {
				pressed = true;
				break;
			}
		}

		if (!pressed) {
			std::cout << "  -> timeout, skip" << std::endl;
			continue;
		}

		// Lire coordonnees jusqu'au relachement
		while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
			ssize_t n = read(evFd, &ev, sizeof(ev));
			if (n != sizeof(ev)) {
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				continue;
			}
			if (ev.type == EV_ABS) {
				if (ev.code == ABS_X) rawX = ev.value;
				else if (ev.code == ABS_Y) rawY = ev.value;
			} else if (ev.type == EV_KEY && ev.code == BTN_TOUCH && ev.value == 0) {
				break;
			}
		}

		// Appliquer la calibration
		int mappedX = static_cast<int>(rawX * cal.scaleX + cal.offsetX);
		int mappedY = static_cast<int>(rawY * cal.scaleY + cal.offsetY);
		double errX = mappedX - static_cast<int>(targets[i].targetX);
		double errY = mappedY - static_cast<int>(targets[i].targetY);
		double errPx = std::sqrt(errX * errX + errY * errY);

		std::cout << "  -> raw (" << rawX << "," << rawY
				  << ") -> calibre (" << mappedX << "," << mappedY
				  << ") erreur = " << static_cast<int>(errPx) << " px" << std::endl;

		// Feedback visuel : croix verte si OK, rouge si erreur > 20px
		if (errPx < 20.0) {
			fb.drawCross(mappedX, mappedY, 8, 0, 255, 0);
		} else {
			fb.drawCross(mappedX, mappedY, 8, 255, 0, 0);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		totalErr += errPx;
		if (errPx > maxErr) maxErr = errPx;
		pointsOk++;
	}

	// Phase de test libre : dessiner pendant 5 secondes
	std::cout << "\n--- Test libre (5 secondes) - toucher l'ecran pour dessiner ---" << std::endl;
	fb.fillScreen(0, 0, 64);

	int rawXfree = 0, rawYfree = 0;
	bool pressedFree = false;
	auto startFree = std::chrono::steady_clock::now();
	struct input_event evFree{};

	while (std::chrono::steady_clock::now() - startFree < std::chrono::seconds(5)) {
		ssize_t n = read(evFd, &evFree, sizeof(evFree));
		if (n != sizeof(evFree)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}

		if (evFree.type == EV_ABS) {
			if (evFree.code == ABS_X) rawXfree = evFree.value;
			else if (evFree.code == ABS_Y) rawYfree = evFree.value;
		} else if (evFree.type == EV_KEY && evFree.code == BTN_TOUCH) {
			pressedFree = (evFree.value != 0);
		} else if (evFree.type == EV_SYN && pressedFree) {
			int mx = static_cast<int>(rawXfree * cal.scaleX + cal.offsetX);
			int my = static_cast<int>(rawYfree * cal.scaleY + cal.offsetY);
			fb.drawCross(mx, my, 5, 255, 255, 0);
		}
	}

	fb.fillScreen(0, 0, 0);
	fb.close();
	::close(evFd);

	if (pointsOk == 0) {
		this->fail("Verification calibration : aucun point mesure");
		return;
	}

	double avgErr = totalErr / pointsOk;
	std::cout << "\n--- Resultats verification ---" << std::endl;
	std::cout << "  Points mesures : " << pointsOk << "/5" << std::endl;
	std::cout << "  Erreur moyenne  : " << static_cast<int>(avgErr) << " px" << std::endl;
	std::cout << "  Erreur max      : " << static_cast<int>(maxErr) << " px" << std::endl;

	if (maxErr < 30.0) {
		this->assert(true, "Calibration valide (err moy " + std::to_string(static_cast<int>(avgErr))
					 + " px, max " + std::to_string(static_cast<int>(maxErr)) + " px)");
	} else {
		this->fail("Calibration imprecise (err max " + std::to_string(static_cast<int>(maxErr))
				   + " px) - relancer testCalibration");
	}
}

void test::ScreenFramebufferManualTest::testCalibration()
{
	const char* evDevice = "/dev/input/event0";

	int evFd = ::open(evDevice, O_RDONLY | O_NONBLOCK);
	if (evFd < 0) {
		std::cerr << "Pas de device tactile " << evDevice
				  << " (skip calibration)" << std::endl;
		this->assert(true, "Calibration skipped (pas de device)");
		return;
	}

	Framebuffer fb;
	if (!fb.open("/dev/fb0")) {
		::close(evFd);
		this->fail("Impossible d'ouvrir /dev/fb0 pour calibration");
		return;
	}

	std::cout << "\n--- Calibration Tactile (3 points) ---" << std::endl;

	// 3 points cibles : haut-gauche, bas-droite, centre
	struct CalPoint {
		uint32_t screenX;
		uint32_t screenY;
		int rawX;
		int rawY;
	};

	const uint32_t margin = 50;
	CalPoint points[3] = {
		{ margin,              margin,               0, 0 },
		{ fb.width - margin,   fb.height - margin,   0, 0 },
		{ fb.width / 2,        fb.height / 2,        0, 0 }
	};

	struct input_absinfo absX{}, absY{};
	ioctl(evFd, EVIOCGABS(ABS_X), &absX);
	ioctl(evFd, EVIOCGABS(ABS_Y), &absY);

	for (int i = 0; i < 3; ++i) {
		fb.fillScreen(0, 0, 0);
		fb.drawCross(points[i].screenX, points[i].screenY, 10, 255, 255, 255);

		std::cout << "Toucher la croix " << (i + 1) << "/3 ("
				  << points[i].screenX << "," << points[i].screenY
				  << ") ..." << std::endl;

		// Purger les evenements en attente
		struct input_event ev{};
		while (read(evFd, &ev, sizeof(ev)) == sizeof(ev)) {}

		// Petite pause pour que l'utilisateur lise le message
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		// Phase 1 : attendre un vrai appui (BTN_TOUCH = 1)
		bool pressed = false;
		int rawX = 0, rawY = 0;
		auto start = std::chrono::steady_clock::now();

		while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
			ssize_t n = read(evFd, &ev, sizeof(ev));
			if (n != sizeof(ev)) {
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				continue;
			}

			if (ev.type == EV_KEY && ev.code == BTN_TOUCH && ev.value == 1) {
				pressed = true;
				break;
			}
		}

		if (!pressed) {
			fb.fillScreen(0, 0, 0);
			fb.close();
			::close(evFd);
			this->fail("Calibration : timeout sur point " + std::to_string(i + 1));
			return;
		}

		// Phase 2 : lire les coordonnees pendant l'appui, puis attendre le relachement
		// On accumule les dernieres valeurs brutes pour avoir une mesure stable
		while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
			ssize_t n = read(evFd, &ev, sizeof(ev));
			if (n != sizeof(ev)) {
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				continue;
			}

			if (ev.type == EV_ABS) {
				if (ev.code == ABS_X) rawX = ev.value;
				else if (ev.code == ABS_Y) rawY = ev.value;
			} else if (ev.type == EV_KEY && ev.code == BTN_TOUCH && ev.value == 0) {
				// Relachement = on a les coordonnees stables
				break;
			}
		}

		points[i].rawX = rawX;
		points[i].rawY = rawY;
		std::cout << "  -> raw (" << rawX << ", " << rawY << ")" << std::endl;

		// Feedback visuel : croix verte
		fb.drawCross(points[i].screenX, points[i].screenY, 10, 0, 255, 0);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	// Calcul des coefficients de calibration (transformation affine 2 points + verification)
	// scaleX = (screenX1 - screenX0) / (rawX1 - rawX0)
	// offsetX = screenX0 - scaleX * rawX0
	double scaleX = 0.0, offsetX = 0.0;
	double scaleY = 0.0, offsetY = 0.0;

	if (points[1].rawX != points[0].rawX && points[1].rawY != points[0].rawY) {
		scaleX = static_cast<double>(points[1].screenX - points[0].screenX)
				 / (points[1].rawX - points[0].rawX);
		offsetX = points[0].screenX - scaleX * points[0].rawX;

		scaleY = static_cast<double>(points[1].screenY - points[0].screenY)
				 / (points[1].rawY - points[0].rawY);
		offsetY = points[0].screenY - scaleY * points[0].rawY;
	}

	std::cout << "\n--- Resultats calibration ---" << std::endl;
	std::cout << "  scaleX  = " << scaleX << std::endl;
	std::cout << "  offsetX = " << offsetX << std::endl;
	std::cout << "  scaleY  = " << scaleY << std::endl;
	std::cout << "  offsetY = " << offsetY << std::endl;
	std::cout << "  formule : screenX = rawX * " << scaleX << " + " << offsetX << std::endl;
	std::cout << "            screenY = rawY * " << scaleY << " + " << offsetY << std::endl;

	// Verification avec le 3e point (centre)
	double verifyX = points[2].rawX * scaleX + offsetX;
	double verifyY = points[2].rawY * scaleY + offsetY;
	double errX = verifyX - points[2].screenX;
	double errY = verifyY - points[2].screenY;
	double errPixels = std::sqrt(errX * errX + errY * errY);

	std::cout << "  verification centre : attendu (" << points[2].screenX << "," << points[2].screenY
			  << ") -> calcule (" << static_cast<int>(verifyX) << "," << static_cast<int>(verifyY)
			  << "), erreur = " << static_cast<int>(errPixels) << " px" << std::endl;

	fb.fillScreen(0, 0, 0);
	fb.close();
	::close(evFd);

	bool ok = (errPixels < 30.0);
	if (ok) {
		// Sauvegarder la calibration pour les prochaines executions
		TouchCalibration cal;
		cal.scaleX = scaleX;
		cal.offsetX = offsetX;
		cal.scaleY = scaleY;
		cal.offsetY = offsetY;
		cal.save();

		this->assert(true, "Calibration OK (erreur " + std::to_string(static_cast<int>(errPixels)) + " px)");
	} else {
		this->fail("Calibration : erreur trop grande (" + std::to_string(static_cast<int>(errPixels)) + " px)");
	}
}
