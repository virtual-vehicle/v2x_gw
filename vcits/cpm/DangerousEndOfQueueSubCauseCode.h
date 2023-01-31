/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/cdd_ts102894_2/ITS-Container.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/cpm -R -no-gen-example -fcompound-names -fno-include-deps -pdu=CPM`
 */

#ifndef	_DangerousEndOfQueueSubCauseCode_H_
#define	_DangerousEndOfQueueSubCauseCode_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum DangerousEndOfQueueSubCauseCode {
	DangerousEndOfQueueSubCauseCode_unavailable	= 0,
	DangerousEndOfQueueSubCauseCode_suddenEndOfQueue	= 1,
	DangerousEndOfQueueSubCauseCode_queueOverHill	= 2,
	DangerousEndOfQueueSubCauseCode_queueAroundBend	= 3,
	DangerousEndOfQueueSubCauseCode_queueInTunnel	= 4
} e_DangerousEndOfQueueSubCauseCode;

/* DangerousEndOfQueueSubCauseCode */
typedef long	 DangerousEndOfQueueSubCauseCode_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DangerousEndOfQueueSubCauseCode;
asn_struct_free_f DangerousEndOfQueueSubCauseCode_free;
asn_struct_print_f DangerousEndOfQueueSubCauseCode_print;
asn_constr_check_f DangerousEndOfQueueSubCauseCode_constraint;
ber_type_decoder_f DangerousEndOfQueueSubCauseCode_decode_ber;
der_type_encoder_f DangerousEndOfQueueSubCauseCode_encode_der;
xer_type_decoder_f DangerousEndOfQueueSubCauseCode_decode_xer;
xer_type_encoder_f DangerousEndOfQueueSubCauseCode_encode_xer;
oer_type_decoder_f DangerousEndOfQueueSubCauseCode_decode_oer;
oer_type_encoder_f DangerousEndOfQueueSubCauseCode_encode_oer;
per_type_decoder_f DangerousEndOfQueueSubCauseCode_decode_uper;
per_type_encoder_f DangerousEndOfQueueSubCauseCode_encode_uper;
per_type_decoder_f DangerousEndOfQueueSubCauseCode_decode_aper;
per_type_encoder_f DangerousEndOfQueueSubCauseCode_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _DangerousEndOfQueueSubCauseCode_H_ */
#include <asn_internal.h>
