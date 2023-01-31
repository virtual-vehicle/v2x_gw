//
// Created by Christoph Pilz on 14.10.2021.
//
// Description:
// Exception, if something is not yet implemented
//
// Author(s): "Christoph Pilz"
// Copyright: "Copyright 2023, vehicleCAPTAIN toolbox"
// Credits: ["Thispointer.com"]
// License: "BSD 3-Clause"
// Version: "1.0"
// Maintainer: "Christoph Pilz"
// E-Mail: "christoph.pilz@v2c2.at"
// Status = "Production"
//
// Possible Improvements:
// - [ ]
//

#ifndef VC_ITS_DECODINGEXCEPTION_H
#define VC_ITS_DECODINGEXCEPTION_H

#pragma once

#include <exception>

class DecodingException : public std::runtime_error
{
public:
    DecodingException(size_t consumed) : std::runtime_error(std::string("Broken encoding at byte: " + std::to_string(consumed))) { };
};

#endif //VC_ITS_DECODINGEXCEPTION_H
