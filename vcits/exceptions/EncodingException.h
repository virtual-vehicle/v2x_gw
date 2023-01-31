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

#ifndef VC_ITS_ENCODINGEXCEPTION_H
#define VC_ITS_ENCODINGEXCEPTION_H

#pragma once

#include <exception>

class EncodingException : public std::runtime_error
{
public:
    EncodingException() : std::runtime_error("Encoding failed: values may be out of range") { };
};

#endif //VC_ITS_ENCODINGEXCEPTION_H
