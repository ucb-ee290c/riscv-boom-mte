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
    def mmte_configID   = 0x7c2
    def mmte_faID       = 0x7c3
    def mmte_fpcID      = 0x7c4
    def mmte_tag_seedID = 0x7c5

    private val mmte_tag_base_min_id = mmte_tag_seedID + 1
    def mmte_tagBaseMaskCount = 3
    def mmte_tagbaseIDs:Seq[Int] = 0.to(mmte_tagBaseMaskCount).map {
        i =>
        mmte_tag_base_min_id + (2 * i)
    }
    def mmte_tagmaskIDs:Seq[Int] = 0.to(mmte_tagBaseMaskCount).map {
        i =>
        mmte_tag_base_min_id + (2 * i) + 1
    }

    def mmte_config_enableShift         =   0
    def mmte_config_enableMask          =   0x7
    def mmte_config_enforceSyncShift    =   3
    def mmte_config_enforceSyncMask     =   0x7
    def mmte_config_permissiveTagShift  =   6
    def mmte_config_permissiveTagMask   =   0xf
}