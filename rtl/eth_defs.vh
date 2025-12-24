`ifndef ETH_DEFS_VH
`define ETH_DEFS_VH

// FPGA identity
`define FPGA_MAC 48'h02_12_34_56_78_9A
`define FPGA_IP 32'h0A0A0A64   // 10.10.10.100

// UDP
`define UDP_LISTEN_PORT 16'd5005

// Hello destination defaults (change to your PC)
`define PC_IP_DEFAULT  32'h0A0A0A01  // 10.10.10.1
`define PC_MAC_DEFAULT 48'hFF_FF_FF_FF_FF_FF  // broadcast until ARP is improved

`endif
