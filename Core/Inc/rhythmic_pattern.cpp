#include "rhythmic_pattern.h"
#include "utils.h"
#include <iostream>
#include <cstdlib>

using namespace std;

const uint8_t prob_hat[20] {
    0, 0, 0, 0, 0,
    1, 1, 1, 1, 1,
    2, 2, 2, 2, 2,
    2, 2, 2, 2, 2
};

const bool rhythms[18][16] = {
    { // son clave
        1, 0, 0, 1,
        0, 0, 1, 0,
        0, 0, 1, 0,
        1, 0, 0, 0
    },
    { // ?
        1, 0, 0, 0,
        1, 0, 0, 0,
        0, 0, 1, 0,
        1, 0, 0, 0
    },
    { // ?
        1, 0, 0, 0,
        0, 0, 0, 1,
        0, 0, 0, 1,
        0, 0, 1, 0
    },
    { // ?
        1, 0, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
        0, 0, 1, 0
    },
    { // ?
        1, 0, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 0,
        0, 0, 1, 0
    },
    { // ?
        1, 0, 0, 0,
        1, 0, 1, 0,
        0, 0, 1, 0,
        1, 0, 0, 0
    },
    { // ?
        1, 0, 0, 0,
        1, 0, 1, 0,
        0, 0, 0, 0,
        1, 0, 0, 0
    },
    { // tresillo
        1, 0, 0, 1,
        0, 0, 1, 0,
        1, 0, 0, 1,
        0, 0, 1, 0
    },
    { // Self
        1, 0, 0, 1,
        0, 0, 1, 0,
        0, 1, 0, 0,
        1, 0, 0, 0
    },
    { // 4x4
        1, 0, 0, 0,
        1, 0, 0, 0,
        1, 0, 0, 0,
        1, 0, 0, 0
    },
    { // 2x4
        1, 0, 0, 0,
        0, 0, 0, 0,
        1, 0, 0, 0,
        0, 0, 0, 0
    },
    { // rumba clave
        1, 0, 0, 1,
        0, 0, 0, 1,
        0, 0, 1, 0,
        1, 0, 0, 0
    },
    { // Morten
        1, 0, 0, 1,
        0, 0, 1, 0,
        0, 1, 0, 0,
        1, 0, 1, 0
    },
    { // Morten
        1, 0, 0, 1,
        0, 0, 1, 0,
        0, 1, 0, 0,
        1, 0, 0, 0
    },
    { // Morten
        1, 0, 0, 0,
        1, 1, 0, 0,
        1, 1, 0, 0,
        1, 0, 0, 0
    },
    { // Morten
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 0
    },
    { // 
        0, 0, 0, 1,
        0, 0, 1, 0,
        0, 0, 1, 0,
        1, 0, 0, 0
    },
    { // 
        0, 0, 0, 1,
        0, 0, 1, 0,
        0, 0, 1, 0,
        1, 0, 0, 0
    },
};

const uint8_t patterns[50][16] {
    {
        78, 82, 38, 66,
        85, 86, 8, 85,
        22, 81, 57, 24,
        37, 2, 2, 62
    },
    {
        84, 59, 47, 57,
        70, 44, 2, 23,
        91, 9, 41, 16,
        69, 21, 74, 56
    },
    {
        22, 80, 32, 14,
        13, 67, 77, 93,
        90, 84, 98, 54,
        35, 36, 96, 22
    },
    {
        33, 24, 66, 73,
        25, 8, 32, 65,
        13, 33, 93, 35,
        28, 34, 5, 76
    },
    {
        22, 31, 43, 34,
        5, 26, 1, 93,
        24, 98, 42, 39,
        31, 34, 22, 78
    },
    {
        58, 95, 32, 13,
        39, 61, 28, 43,
        65, 65, 19, 78,
        57, 48, 96, 28
    },
    {
        59, 15, 29, 74,
        32, 33, 23, 10,
        23, 5, 95, 20,
        20, 61, 6, 76
    },
    {
        97, 12, 77, 47,
        26, 43, 20, 57,
        15, 26, 91, 33,
        27, 7, 59, 54
    },
    {
        21, 88, 19, 95,
        66, 96, 33, 90,
        82, 1, 60, 56,
        77, 4, 71, 85
    },
    {
        4, 20, 42, 43,
        63, 14, 34, 69,
        90, 27, 13, 68,
        39, 52, 68, 25
    },
    {
        89, 19, 40, 41,
        29, 80, 28, 59,
        59, 61, 78, 56,
        71, 81, 52, 99
    },
    {
        94, 17, 77, 44,
        31, 40, 60, 39,
        16, 70, 30, 45,
        83, 55, 97, 67
    },
    {
        13, 33, 27, 65,
        86, 49, 95, 25,
        12, 76, 53, 57,
        38, 50, 77, 68
    },
    {
        65, 94, 22, 70,
        59, 58, 75, 14,
        63, 21, 1, 10,
        12, 63, 60, 84
    },
    {
        99, 99, 100, 90,
        55, 5, 61, 76,
        38, 92, 8, 47,
        82, 65, 92, 94
    },
    {
        82, 52, 93, 82,
        43, 93, 76, 97,
        75, 96, 66, 25,
        25, 92, 58, 69
    },
    {
        66, 74, 9, 39,
        54, 15, 35, 19,
        33, 77, 91, 95,
        8, 87, 66, 69
    },
    {
        53, 80, 30, 44,
        38, 3, 28, 60,
        82, 59, 11, 24,
        42, 34, 100, 11
    },
    {
        96, 77, 79, 44,
        25, 62, 54, 73,
        29, 99, 96, 58,
        62, 56, 98, 59
    },
    {
        81, 46, 25, 70,
        61, 26, 34, 5,
        97, 72, 73, 91,
        81, 97, 95, 7
    },
    {
        8, 70, 17, 66,
        46, 0, 21, 93,
        51, 21, 78, 13,
        8, 62, 70, 70
    },
    {
        23, 29, 48, 87,
        92, 90, 46, 17,
        38, 43, 2, 87,
        47, 18, 97, 49
    },
    {
        70, 19, 12, 0,
        15, 65, 56, 13,
        92, 32, 98, 40,
        61, 40, 6, 70
    },
    {
        34, 49, 46, 100,
        74, 45, 33, 9,
        20, 62, 72, 59,
        4, 40, 95, 70
    },
    {
        92, 48, 36, 11,
        27, 96, 37, 9,
        72, 2, 68, 14,
        100, 67, 14, 31
    },
    {
        66, 62, 14, 8,
        90, 19, 52, 58,
        44, 29, 27, 15,
        60, 99, 88, 18
    },
    {
        11, 22, 6, 41,
        78, 48, 57, 37,
        15, 35, 73, 10,
        52, 67, 85, 95
    },
    {
        8, 28, 77, 73,
        93, 14, 54, 95,
        51, 41, 37, 97,
        64, 15, 28, 14
    },
    {
        58, 25, 0, 22,
        22, 94, 2, 58,
        44, 49, 10, 41,
        66, 56, 14, 28
    },
    {
        65, 99, 31, 3,
        69, 58, 55, 52,
        97, 56, 24, 72,
        3, 46, 49, 1
    },
    {
        53, 69, 42, 7,
        100, 25, 80, 19,
        83, 41, 5, 42,
        70, 20, 21, 8
    },
    {
        70, 57, 3, 1,
        47, 9, 70, 28,
        29, 26, 94, 83,
        21, 99, 65, 38
    },
    {
        53, 61, 78, 88,
        0, 52, 99, 7,
        92, 11, 23, 98,
        10, 90, 9, 22
    },
    {
        12, 68, 61, 8,
        28, 4, 63, 30,
        71, 77, 76, 13,
        78, 51, 69, 48
    },
    {
        81, 2, 21, 56,
        72, 83, 30, 49,
        1, 66, 75, 8,
        68, 23, 22, 80
    },
    {
        98, 40, 18, 82,
        21, 33, 59, 10,
        56, 44, 9, 98,
        72, 15, 78, 67
    },
    {
        49, 94, 4, 15,
        49, 55, 89, 50,
        69, 1, 13, 35,
        91, 24, 69, 52
    },
    {
        36, 21, 29, 17,
        62, 33, 33, 60,
        91, 83, 77, 80,
        24, 32, 91, 85
    },
    {
        49, 74, 46, 71,
        27, 6, 50, 87,
        27, 83, 76, 37,
        30, 79, 37, 53
    },
    {
        25, 92, 87, 17,
        42, 56, 80, 39,
        89, 83, 93, 58,
        34, 83, 32, 28
    },
    {
        42, 81, 39, 27,
        41, 4, 84, 45,
        71, 0, 51, 80,
        44, 8, 83, 19
    },
    {
        90, 92, 8, 70,
        82, 54, 13, 48,
        43, 10, 3, 22,
        66, 16, 81, 98
    },
    {
        14, 84, 32, 77,
        33, 26, 78, 62,
        70, 45, 33, 6,
        63, 39, 9, 44
    },
    {
        57, 61, 93, 20,
        53, 45, 87, 54,
        92, 14, 11, 38,
        98, 19, 88, 8
    },
    {
        20, 59, 99, 41,
        76, 15, 85, 6,
        16, 13, 42, 24,
        46, 94, 82, 26
    },
    {
        59, 27, 4, 91,
        11, 58, 69, 38,
        70, 57, 50, 52,
        86, 1, 35, 18
    },
    {
        78, 16, 83, 41,
        67, 13, 92, 48,
        43, 72, 40, 17,
        30, 95, 92, 67
    },
    {
        14, 83, 90, 4,
        51, 95, 94, 1,
        50, 40, 60, 46,
        91, 75, 42, 20
    },
    {
        50, 54, 92, 74,
        29, 44, 26, 9,
        40, 83, 76, 94,
        70, 63, 23, 99
    },
    {
        71, 25, 49, 50,
        41, 29, 29, 4,
        89, 85, 46, 30,
        15, 23, 51, 15
    }
};
