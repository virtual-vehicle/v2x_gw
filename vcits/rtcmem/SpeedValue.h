/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/cdd_ts102894_2/ITS-Container.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/rtcmem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=RTCMEM`
 */

#ifndef	_SpeedValue_H_
#define	_SpeedValue_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SpeedValue {
	SpeedValue_standstill	= 0,
	SpeedValue_oneCentimeterPerSec	= 1,
	SpeedValue_unavailable	= 16383
} e_SpeedValue;

/* SpeedValue */
typedef long	 SpeedValue_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_SpeedValue_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_SpeedValue;
asn_struct_free_f SpeedValue_free;
asn_struct_print_f SpeedValue_print;
asn_constr_check_f SpeedValue_constraint;
ber_type_decoder_f SpeedValue_decode_ber;
der_type_encoder_f SpeedValue_encode_der;
xer_type_decoder_f SpeedValue_decode_xer;
xer_type_encoder_f SpeedValue_encode_xer;
oer_type_decoder_f SpeedValue_decode_oer;
oer_type_encoder_f SpeedValue_encode_oer;
per_type_decoder_f SpeedValue_decode_uper;
per_type_encoder_f SpeedValue_encode_uper;
per_type_decoder_f SpeedValue_decode_aper;
per_type_encoder_f SpeedValue_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _SpeedValue_H_ */
#include <asn_internal.h>
