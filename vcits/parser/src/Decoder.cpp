//
// Created by christophpilz on 19.05.2021.
//

#include "Decoder.h"

#include <iostream>

extern "C" {
#include "asn_application.h"
}

#include "exceptions/V2XLibExceptions.h"

void *Decoder::decode(const asn_TYPE_descriptor_t *type_descriptor, const void *buffer, size_t buffer_size) {
    void *decoded_object = 0;
    const asn_dec_rval_t return_value = uper_decode(0, type_descriptor, &decoded_object, buffer, buffer_size, 0, 0);

    if (return_value.code != RC_OK)
        throw DecodingException(return_value.consumed);

    return decoded_object;
}