#pragma once

#include <Adafruit_NeoPixel.h>
#include <FastLED.h>
#include <mp/rendering/shape.hpp>
#include <mp/utility/maths.hpp>



struct HSV : public Vec<3, uint8_t>
{
    using Vec<3, uint8_t>::Vec;
    uint8_t &h = x();
    uint8_t &s = y();
    uint8_t &v = z();
};

class Renderer
{
public:
    Renderer(int nPixels, float physMin, float physMax, Adafruit_NeoPixel &leds)
        : nPixels(nPixels)
        , phys2pix(physMin, physMax, 0, nPixels)
        , xWrap(physMax) 
        , leds(leds)
    {}
    
    using Line = mp::Line<2, float>;
    void drawShape(const Line &line)
    {
        static mp::map_logistic<float> y2byte(-0.05f, 0.05f, 32, 108);
        static mp::map_logistic<float> theta2hue(-4, 4, 130, 140);

        float x0 = line.vertices[0].x();
        float x1 = line.vertices[1].x();
        float y0 = line.vertices[0].y();
        float y1 = line.vertices[1].y();
        float dy = y1 - y0;
        int start = phys2pix(x0);
        int end = phys2pix(x1);

        float gradient = dy ? dy / (x1 - x0) : 0.0;
        uint8_t hue = theta2hue(atanf(gradient));
        float inc = dy / (float)(end - start);
        float y = y0;
        for (int i = start; i < end; i++)
        {
            uint8_t b = y2byte(y);
            CHSV hsv(hue, 255 - b, b);
            CRGB rgb = hsv;
            y += inc;
            leds.setPixelColor(i, rgb.r, rgb.g, rgb.b);
        }
    }

    void show()
    {
        leds.show();
    }
protected:
    void drawLine(Vec<1, float> p1, Vec<1, float> p2, CRGB colour)
    {
        for (int i = p1.x(); i < static_cast<int>(p2.x()); ++i)
            setPixel(i % nPixels, colour);
    }

    void setPixel(int p, CRGB rgb)
    {
        leds.setPixelColor(p, rgb.r, rgb.g, rgb.b);        
    }

    size_t nPixels;
    mp::map_linear<float> phys2pix;
    mp::wrapped_distance<float> xWrap;
    Adafruit_NeoPixel &leds;
};
