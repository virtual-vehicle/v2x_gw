/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ElectronicRegistrationIdentificationVehicleDataModule"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/is_ts103301/iso-patched/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/mapem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=MAPEM`
 */

#ifndef	_EuVehicleCategoryN_H_
#define	_EuVehicleCategoryN_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum EuVehicleCategoryN {
	EuVehicleCategoryN_n1	= 0,
	EuVehicleCategoryN_n2	= 1,
	EuVehicleCategoryN_n3	= 2
} e_EuVehicleCategoryN;

/* EuVehicleCategoryN */
typedef long	 EuVehicleCategoryN_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_EuVehicleCategoryN_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_EuVehicleCategoryN;
extern const asn_INTEGER_specifics_t asn_SPC_EuVehicleCategoryN_specs_1;
asn_struct_free_f EuVehicleCategoryN_free;
asn_struct_print_f EuVehicleCategoryN_print;
asn_constr_check_f EuVehicleCategoryN_constraint;
ber_type_decoder_f EuVehicleCategoryN_decode_ber;
der_type_encoder_f EuVehicleCategoryN_encode_der;
xer_type_decoder_f EuVehicleCategoryN_decode_xer;
xer_type_encoder_f EuVehicleCategoryN_encode_xer;
oer_type_decoder_f EuVehicleCategoryN_decode_oer;
oer_type_encoder_f EuVehicleCategoryN_encode_oer;
per_type_decoder_f EuVehicleCategoryN_decode_uper;
per_type_encoder_f EuVehicleCategoryN_encode_uper;
per_type_decoder_f EuVehicleCategoryN_decode_aper;
per_type_encoder_f EuVehicleCategoryN_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _EuVehicleCategoryN_H_ */
#include <asn_internal.h>
