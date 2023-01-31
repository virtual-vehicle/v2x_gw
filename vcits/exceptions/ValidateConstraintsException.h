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

#ifndef VC_ITS_VALIDATECONSTRAINTSEXCEPTION_H
#define VC_ITS_VALIDATECONSTRAINTSEXCEPTION_H

#pragma once

#include <exception>

class ValidateConstraintsException : public std::logic_error
{
public:
    ValidateConstraintsException() : std::logic_error("Validation failed: parameters may be defined wrong") { };
};

#endif //VC_ITS_VALIDATECONSTRAINTSEXCEPTION_H
