#
# RegionalExtension.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
#
# The formal definition used by each regional extension point follows the style of a unique indexing
# integer and a type definition bound into a set.

# RegionalExtension {REG-EXT-ID-AND-TYPE : Set} ::= SEQUENCE {
# regionId REG-EXT-ID-AND-TYPE.&id( {Set} ),
# regExtValue REG-EXT-ID-AND-TYPE.&Type( {Set} {@regionId} )
# }

# regional #TODO: RegionalExtensions are not yet implemented in asn1c
