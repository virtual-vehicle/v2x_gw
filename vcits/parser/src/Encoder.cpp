//
// Created by christophpilz on 19.05.2021.
//

#include "Encoder.h"

#include <iostream>

extern "C" {
#include "asn_application.h"
}

#include "exceptions/V2XLibExceptions.h"

bool Encoder::validate_constraints(asn_TYPE_descriptor_t *type_descriptor, const void *struct_ptr) {
    char error_buffer[128];
    size_t error_length = sizeof(error_buffer);
    const int return_value = asn_check_constraints(type_descriptor, struct_ptr, error_buffer, &error_length);

    if (return_value != 0)
        throw ValidateConstraintsException();

    return (return_value == 0);
}

ssize_t Encoder::encode(const asn_TYPE_descriptor_t *type_descriptor, const asn_per_constraints_t *constraints,
                        const void *struct_ptr, void **buffer) {
    const ssize_t error_code = uper_encode_to_new_buffer(type_descriptor, constraints, struct_ptr, buffer);

    if (error_code == -1)
        throw EncodingException();

    return error_code;
}