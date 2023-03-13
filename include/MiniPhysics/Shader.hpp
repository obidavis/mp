#pragma once

#include "Vec.hpp"


template <int Dim, typename T>
struct PointLight 
{
    Vec<3, T> colour;
    T ambientIntensity;
    T diffuseIntensity;
    Vec<Dim, T> position;
    struct
    {
        T constant;
        T linear;
        T exp;
    } attenuation;
};

template <int Dim, typename T>
Vec<Dim, T> reflect(Vec<Dim, T> incident, Vec<Dim, T> normal) 
{
    return incident - 2.0 * Vec<Dim, T>::dot(normal, incident) * normal;
}

template <int Dim, typename T>
Vec<4, T> _calculateLight(PointLight<Dim, T> light, Vec<Dim, T> lightDirection, Vec<Dim, T> normal, Vec<Dim, T> eyePos, Vec<Dim, T> surfacePos)
{
    Vec<3, T> ambient = light.colour * light.ambientIntensity;

    Vec<Dim, T> norm = normal.normalised();
    T diff = std::max(Vec<Dim, T>::dot(norm, lightDirection), 0.0);
    Vec<3, T> diffuse = diff * light.diffuseIntensity * light.colour;
    
    Vec<3, T> vertexToEye = (eyePos - surfacePos).normalised();
    Vec<3, T> reflectDir = reflect(-lightDirection, norm);
    T specularfactor = Vec<3, T>::dot(vertexToEye, reflectDir);
    specularfactor = std::pow(specularfactor, 32);
    Vec<3, T> specular = light.colour * specularfactor * 0.5;

    Vec<3, T> colour = ambient + diffuse + specular;
    for (int i = 0; i < 3; ++i)
        colour[i] = std::min(colour[i], 1.0);
    return {colour[0], colour[1], colour[2], 1.0};
}
