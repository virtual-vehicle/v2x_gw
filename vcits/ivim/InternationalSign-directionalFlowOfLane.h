/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "GDD"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/is_ts103301/iso-patched/ISO14823-missing.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/ivim -R -no-gen-example -fcompound-names -fno-include-deps -pdu=IVIM`
 */

#ifndef	_InternationalSign_directionalFlowOfLane_H_
#define	_InternationalSign_directionalFlowOfLane_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum InternationalSign_directionalFlowOfLane {
	InternationalSign_directionalFlowOfLane_sDL	= 1,
	InternationalSign_directionalFlowOfLane_sLT	= 2,
	InternationalSign_directionalFlowOfLane_sRT	= 3,
	InternationalSign_directionalFlowOfLane_lTO	= 4,
	InternationalSign_directionalFlowOfLane_rTO	= 5,
	InternationalSign_directionalFlowOfLane_cLL	= 6,
	InternationalSign_directionalFlowOfLane_cRI	= 7,
	InternationalSign_directionalFlowOfLane_oVL	= 8
} e_InternationalSign_directionalFlowOfLane;

/* InternationalSign-directionalFlowOfLane */
typedef long	 InternationalSign_directionalFlowOfLane_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_InternationalSign_directionalFlowOfLane_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_InternationalSign_directionalFlowOfLane;
asn_struct_free_f InternationalSign_directionalFlowOfLane_free;
asn_struct_print_f InternationalSign_directionalFlowOfLane_print;
asn_constr_check_f InternationalSign_directionalFlowOfLane_constraint;
ber_type_decoder_f InternationalSign_directionalFlowOfLane_decode_ber;
der_type_encoder_f InternationalSign_directionalFlowOfLane_encode_der;
xer_type_decoder_f InternationalSign_directionalFlowOfLane_decode_xer;
xer_type_encoder_f InternationalSign_directionalFlowOfLane_encode_xer;
oer_type_decoder_f InternationalSign_directionalFlowOfLane_decode_oer;
oer_type_encoder_f InternationalSign_directionalFlowOfLane_encode_oer;
per_type_decoder_f InternationalSign_directionalFlowOfLane_decode_uper;
per_type_encoder_f InternationalSign_directionalFlowOfLane_encode_uper;
per_type_decoder_f InternationalSign_directionalFlowOfLane_decode_aper;
per_type_encoder_f InternationalSign_directionalFlowOfLane_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _InternationalSign_directionalFlowOfLane_H_ */
#include <asn_internal.h>
