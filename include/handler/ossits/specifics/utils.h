//
// Created by Alina Steinberger
//
// Description:
// This uitls file contains a few supporting methods for copying message parts
//
// Author(s): "Alina Steinberger"
// Copyright: "Copyright 2023, vehicleCAPTAIN toolbox"
// Credits: ["Alina Steinberger"]
// License: "BSD-3-clause"
// Version: "1.0.0"
// Maintainer: "Alina Steinberger"
// E-Mail: "alina.steinberger@v2c2.at"
// Status = "Production"
//
// Reference to used code:
// <Description1 what was used> (<Link>)
// <Description2 what was used> (<Link>)
//
// Possible Improvements:
// [] <Bug 1>
// [] <Refactoring Idea 2>
// [] <Feature Idea 3>
//

extern "C" {
#include "ossits/include/ossits/ossits.h"
}
namespace utils
{
    class ASN1Exception {
    private:
        int code;
    public:
        ASN1Exception(int asn1_code);
        ASN1Exception(const ASN1Exception & that);
        int get_code() const;
    };

    ASN1Exception::ASN1Exception(int asn1_code)
    {
        code = asn1_code;
    }

    ASN1Exception::ASN1Exception(const ASN1Exception & that)
    {
        code = that.code;
    }

    int ASN1Exception::get_code() const
    {
        return code;
    }

    /*
    * The ASN.1/C++ error reporting function.
    */

    void throw_error(int code)
    {
        throw ASN1Exception(code);
    }
}