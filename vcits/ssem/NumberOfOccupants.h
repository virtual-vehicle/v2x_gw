/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/cdd_ts102894_2/ITS-Container.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/ssem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=SSEM`
 */

#ifndef	_NumberOfOccupants_H_
#define	_NumberOfOccupants_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum NumberOfOccupants {
	NumberOfOccupants_oneOccupant	= 1,
	NumberOfOccupants_unavailable	= 127
} e_NumberOfOccupants;

/* NumberOfOccupants */
typedef long	 NumberOfOccupants_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_NumberOfOccupants;
asn_struct_free_f NumberOfOccupants_free;
asn_struct_print_f NumberOfOccupants_print;
asn_constr_check_f NumberOfOccupants_constraint;
ber_type_decoder_f NumberOfOccupants_decode_ber;
der_type_encoder_f NumberOfOccupants_encode_der;
xer_type_decoder_f NumberOfOccupants_decode_xer;
xer_type_encoder_f NumberOfOccupants_encode_xer;
oer_type_decoder_f NumberOfOccupants_decode_oer;
oer_type_encoder_f NumberOfOccupants_encode_oer;
per_type_decoder_f NumberOfOccupants_decode_uper;
per_type_encoder_f NumberOfOccupants_encode_uper;
per_type_decoder_f NumberOfOccupants_decode_aper;
per_type_encoder_f NumberOfOccupants_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _NumberOfOccupants_H_ */
#include <asn_internal.h>
