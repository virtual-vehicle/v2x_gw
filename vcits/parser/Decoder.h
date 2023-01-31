//
// Created by christophpilz on 19.05.2021.
//
// Description:
// Decoder Wrapper for C++
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
// - [] have a decode method without type descriptor -> automatically find out which type of message it is and decode the right one
// - [] refactor C++ style
//

#ifndef VC_ITS_LIB_DECODER_H
#define VC_ITS_LIB_DECODER_H

extern "C" {
#include "asn_application.h"
}

class Decoder {
public:
    /// decode an asn1 bitstream to receive a V2X message object
    /// \param [in] type_descriptor V2X message type
    /// \param [in] buffer V2X message as asn1 bitstream
    /// \param [in] buffer_size the number of bytes in the buffer
    /// \return [out] the V2X message struct
    static void *decode(const asn_TYPE_descriptor_t *type_descriptor, const void *buffer, size_t buffer_size);

private:
    Decoder() {};
};


#endif //VC_ITS_LIB_DECODER_H
