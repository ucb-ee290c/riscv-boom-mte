package boom.common
import chisel3._
import chisel3.util._
import freechips.rocketchip.tile.CustomCSR

object MTEConfig {
    val tagBits:Int = 4
    /** The number of bytes which a single tag covers */
    val taggingGranuleBytes:Int = 16
}

object MTEInstructions {
/*
    * The custom-3 RISCV space uses opcode 1111011. We are free to dice it 
    * however we see fit, but we follow (roughly) the default convention for
    * reuse of Rd, Rs1, Rs2 decoding.
    * 
    * Conventions:
    * R-type uses funct3 = 3'b111 and funct7 to differentiate ops
    */

    /* R-types */
    /* mte.add is funct7 = 7'b0 */
    def MTE_ADD            = BitPat("b0000000_?????_?????_111_?????_1111011")
    /* mte.irt is funct7 = 7'b1 */
    def MTE_IRT            = BitPat("b0000001_?????_?????_111_?????_1111011")
    /* S-types */
    /* mte.stti is funct3 = 3'b101 */
    def MTE_STTI           = BitPat("b????????????_?????_101_?????_1111011")
    /* I-types */
    /* mte.addti is funct3 = 3'b110 */
    def MTE_ADDTI          = BitPat("b????????????_?????_110_?????_1111011")
}

object MTECSRs {

    /* 
    All registers are allocated from the custom machine RW space
    (Table 2.1: Allocation of RISC-V CSR address ranges.)
    */
    val smte_configID   = 0x5c0
    val smte_fstatusID  = 0x5c1
    val smte_faID       = 0x5c2

    private val smte_tag_base_min_id = 0x5c3
    private val mte_region_max = 5
    private val smte_tag_base_max_id = smte_tag_base_min_id + mte_region_max * 2 - 1

    def smte_tagbaseIDs(mteRegions:List[BoomMTERegion]):Seq[Int] = {
        val regionCount = mteRegions.size
        require(regionCount <= mte_region_max, s"Too many MTE regions (requested=$regionCount, max=$mte_region_max)")
        0.until(regionCount).map {
            i =>
            smte_tag_base_min_id + (2 * i)
        }
    }

    def smte_tagmaskIDs(mteRegions:List[BoomMTERegion]):Seq[Int] = {
        val regionCount = mteRegions.size
        require(regionCount <= mte_region_max, s"Too many MTE regions (requested=$regionCount, max=$mte_region_max)")

        0.until(regionCount).map {
            i =>
            smte_tag_base_min_id + (2 * i) + 1
        }
    }

    def smte_dbg_tcache_req   = 0x5cd
    def smte_dbg_tcache_data0 = 0x5ce 
    def smte_dbg_tcache_data1 = 0x5cf
    
    assert(smte_dbg_tcache_req == smte_tag_base_max_id + 1)

    def widthToMask(width:Int):BigInt = {
        (BigInt(1) << width) - 1
    }
    def smte_config_enableShift          =   0
    def smte_config_enableWidth          =   4
    def smte_config_enforceSyncShift     =   smte_config_enableShift + smte_config_enableWidth
    def smte_config_enforceSyncWidth     =   4
    def smte_config_permissiveTagShift   =   smte_config_enforceSyncShift + smte_config_enforceSyncWidth
    def smte_config_permissiveTagWidth   =   4

    def smte_fstatus_validShift         =  0
    def smte_fstatus_validWidth         =  1
    def smte_fstatus_addressTagShift = smte_fstatus_validShift + smte_fstatus_validWidth
    def smte_fstatus_addressTagWidth = MTEConfig.tagBits
    def smte_fstatus_physicalTagShift =  smte_fstatus_addressTagShift + smte_fstatus_addressTagWidth
    def smte_fstatus_physicalTagWidth =  MTEConfig.tagBits
    def smte_fstatus_privShift = smte_fstatus_physicalTagShift + smte_fstatus_physicalTagWidth
    def smte_fstatus_privWidth = 2
    def smte_fstatus_isLoadShift = smte_fstatus_privShift + smte_fstatus_privWidth
    def smte_fstatus_isLoadWidth = 1
    def smte_fstatus_opSizeShift = smte_fstatus_isLoadShift + smte_fstatus_isLoadWidth
    def smte_fstatus_opSizeWidth = 2
}