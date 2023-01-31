/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/is_ts103301/iso-patched/ISO14906(2018)EfcDsrcApplicationv6-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/ivim -R -no-gen-example -fcompound-names -fno-include-deps -pdu=IVIM`
 */

#ifndef	_VehicleSpecificCharacteristics_H_
#define	_VehicleSpecificCharacteristics_H_


#include <asn_application.h>

/* Including external dependencies */
#include "EnvironmentalCharacteristics.h"
#include "EngineCharacteristics.h"
#include "DescriptiveCharacteristics.h"
#include "FutureCharacteristics.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* VehicleSpecificCharacteristics */
typedef struct VehicleSpecificCharacteristics {
	EnvironmentalCharacteristics_t	 environmentalCharacteristics;
	EngineCharacteristics_t	 engineCharacteristics;
	DescriptiveCharacteristics_t	 descriptiveCharacteristics;
	FutureCharacteristics_t	 futureCharacteristics;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} VehicleSpecificCharacteristics_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_VehicleSpecificCharacteristics;

#ifdef __cplusplus
}
#endif

#endif	/* _VehicleSpecificCharacteristics_H_ */
#include <asn_internal.h>
