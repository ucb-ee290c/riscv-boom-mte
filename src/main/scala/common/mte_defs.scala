package boom.common
import chisel3.util._
import freechips.rocketchip.tile.CustomCSR

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

     /* I-types */
     /* mte.addti is funct3 = 3'b110 */
     def MTE_ADDTI          = BitPat("b????????????_?????_110_?????_1111011")
//   def CUSTOM3            = BitPat("b?????????????????000?????1111011")
//   def CUSTOM3_RS1        = BitPat("b?????????????????010?????1111011")
//   def CUSTOM3_RS1_RS2    = BitPat("b?????????????????011?????1111011")
//   def CUSTOM3_RD         = BitPat("b?????????????????100?????1111011")
//   def CUSTOM3_RD_RS1     = BitPat("b?????????????????110?????1111011")
//   def CUSTOM3_RD_RS1_RS2 = BitPat("b?????????????????111?????1111011")
}

// class MMTEConfigCSR(implicit p: Parameters) extends BoomBundle {
//     val enable          = UInt(3.W)
//     val enforce_sync    = UInt(3.W)
//     val permissive_tag  = UInt(4.W)
// }

object MTECSRs {

    /* 
    All registers are allocated from the custom machine RW space
    (Table 2.1: Allocation of RISC-V CSR address ranges.)
    */
    def smte_configID   = 0x5c0
    def smte_faID       = 0x5c1
    def smte_fpcID      = 0x5c2
    def smte_tag_seedID = 0x5c3

    private val smte_tag_base_min_id = smte_tag_seedID + 1
    //TODO: Plumb the tag base/mask out to a broader config
    def smte_tagBaseMaskCount = 3
    def smte_tagbaseIDs:Seq[Int] = 0.to(smte_tagBaseMaskCount).map {
        i =>
        smte_tag_base_min_id + (2 * i)
    }
    def smte_tagmaskIDs:Seq[Int] = 0.to(smte_tagBaseMaskCount).map {
        i =>
        smte_tag_base_min_id + (2 * i) + 1
    }

    def smte_config_enableShift         =   0
    def smte_config_enableMask          =   0x7
    def smte_config_enforceSyncShift    =   3
    def smte_config_enforceSyncMask     =   0x7
    def smte_config_permissiveTagShift  =   6
    def smte_config_permissiveTagMask   =   0xf
}