/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/cdd_ts102894_2/ITS-Container.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/ivim -R -no-gen-example -fcompound-names -fno-include-deps -pdu=IVIM`
 */

#ifndef	_StationarySince_H_
#define	_StationarySince_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum StationarySince {
	StationarySince_lessThan1Minute	= 0,
	StationarySince_lessThan2Minutes	= 1,
	StationarySince_lessThan15Minutes	= 2,
	StationarySince_equalOrGreater15Minutes	= 3
} e_StationarySince;

/* StationarySince */
typedef long	 StationarySince_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_StationarySince;
asn_struct_free_f StationarySince_free;
asn_struct_print_f StationarySince_print;
asn_constr_check_f StationarySince_constraint;
ber_type_decoder_f StationarySince_decode_ber;
der_type_encoder_f StationarySince_encode_der;
xer_type_decoder_f StationarySince_decode_xer;
xer_type_encoder_f StationarySince_encode_xer;
oer_type_decoder_f StationarySince_decode_oer;
oer_type_encoder_f StationarySince_encode_oer;
per_type_decoder_f StationarySince_decode_uper;
per_type_encoder_f StationarySince_encode_uper;
per_type_decoder_f StationarySince_decode_aper;
per_type_encoder_f StationarySince_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _StationarySince_H_ */
#include <asn_internal.h>
