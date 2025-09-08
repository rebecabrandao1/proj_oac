//o que faz cada função:
//top: é uma espécie de main que vai chamar o restante dos módulos. integra CPU single-cycle RISC-V, memória de instruções e memória de dados [imem() e dmem()].

//riscvsingle: Integra controle e caminho de dados de um processador RISC-V single-cycle: recebe instrução e dado de memória, gera sinais de controle, executa a operação da ALU, calcula próximo PC, define endereço/dado de acesso à memória de dados e decide qual valor escrever de volta nos registradores.

//controller: O módulo controller pega campos da instrução (opcode = op, funct3, bit do funct7) e o sinal Zero da ALU, e gera todos os sinais de controle para o datapath em um único ciclo. Ele é dividido em dois subdecs: maindec: decide coisas “macro” (ResultSrc, MemWrite, Branch, ALUSrc, RegWrite, Jump, ImmSrc, ALUOp) e aludec: a partir de ALUOp + funct3 + bit do funct7 gera ALUControl (qual operação a ALU faz).

//maindec: Decodifica o opcode e gera o pacote de sinais de alto nível: escreve registrador? tipo de imediato? seleção da fonte B da ALU? escreve memória? de onde vem o resultado para write-back? é branch? é jump? e qual ALUOp (que refina depois na aludec).
//aludec:  Refina a operação da ALU usando ALUOp + funct3 + (funct7b5 para distinguir add/sub). Retorna ALUControl com o código da operação específica.
//datapath: Interliga PC, geração de próximo PC (sequencial ou branch/jump), banco de registradores, extensão de imediato, multiplexadores, ALU e seleção final do valor escrito de volta.
//regfile: Banco de 32 registradores de 32 bits, leitura combinacional de duas portas e escrita síncrona em uma porta. x0 é fixo em 0.
//extend: Faz a extensão (sign-extend) dos imediatos de acordo com o formato (I, S, B, J). Usa ImmSrc para selecionar o tipo correto.
//flopr: Flip-flop com reset assíncrono parametrizável em largura. Armazena PC (e pode ser reutilizado para outros registros de estado).
//mux2: Multiplexador de 2 entradas parametrizável em largura.
//mux3: Multiplexador de 3 entradas parametrizável (usa codificação s[1]?d2: ...). Aqui escolhe entre ALUResult, ReadData (load) ou PC+4 (para JAL).
//dmem: Memória de dados simples (RAM) word-aligned. Leitura combinacional, escrita síncrona quando MemWrite=1.
//imem: Memória de instruções somente leitura (carregada via $readmemh). Endereçamento word-aligned usando a[31:2].
//alu: Executa operações aritméticas/lógicas básicas e define sinal zero. Também calcula overflow para add/sub e implementa slt via lógica de sinal/overflow.
//o que foi mudado:
//7'b0110011: controls = 11'b1_00_0_0_00_0_10_0; // R-type  //add 010 
//7'b1101111: controls = 11'b1_11_0_0_10_0_00_1; // jal -> Jump=1, ResultSrc=PC+4, ImmSrc=J
// mux3 #(32) resultmux(ALUResult, ReadData, PCPlus4, ResultSrc, Result); //troca PCPlus4 pra salvar no JAL
// assign PCSrcFinal = PCSrc | Jump; // permite jal (salto incondicional)
// Passa sinal Jump do controller para o datapath (riscvsingle -> datapath)
// extend(): adicionada forma J (immJ) + seleção via ImmSrc=2'b11
// regfile: evita escrita em x0 (if (we3 && a3!=0))
// default do maindec sem 'x' pra não poluir waveform
//Módulos com mudanças: maindec; datapath; riscvsingle; extend (e pequeno ajuste em regfile)
module top(
  input  logic        clk, reset, //entradas: clock, reset, 

  output logic [31:0] WriteData, DataAdr,//referencia o que será escrito e endereçado na memória

  output logic        MemWrite
);

  logic [31:0] PC, Instr, ReadData; //declarando os módulos usados no circuito

  
  // Instancia CPU e memórias
  riscvsingle rvsingle(
    .clk(clk), .reset(reset),
    .PC(PC),
    .Instr(Instr),
    .MemWrite(MemWrite),
    .ALUResult(DataAdr), // ALUResult = endereço de dados (DataAdr)
    .WriteData(WriteData),
    .ReadData(ReadData)
  );

  imem imem(PC, Instr);
  dmem dmem(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule

module riscvsingle(
  input  logic        clk, reset,
  output logic [31:0] PC,
  input  logic [31:0] Instr,
  output logic        MemWrite,
  output logic [31:0] ALUResult, WriteData,
  input  logic [31:0] ReadData
);

  logic       ALUSrc, RegWrite, Jump, Zero, PCSrc;
  logic [1:0] ResultSrc, ImmSrc;
  logic [2:0] ALUControl;

  controller c(
    .op(Instr[6:0]),
    .funct3(Instr[14:12]),
    .funct7b5(Instr[30]),
    .Zero(Zero),
    .ResultSrc(ResultSrc),
    .MemWrite(MemWrite),
    .PCSrc(PCSrc),
    .ALUSrc(ALUSrc),
    .RegWrite(RegWrite),
    .Jump(Jump),
    .ImmSrc(ImmSrc),
    .ALUControl(ALUControl)
  );

  datapath dp(
    .clk(clk), .reset(reset),
    .ResultSrc(ResultSrc),
    .PCSrc(PCSrc),
    .ALUSrc(ALUSrc),
    .RegWrite(RegWrite),
    .Jump(Jump),           
    .ImmSrc(ImmSrc),
    .ALUControl(ALUControl),
    .Zero(Zero),
    .PC(PC),
    .Instr(Instr),
    .ALUResult(ALUResult),
    .WriteData(WriteData),
    .ReadData(ReadData)
  );
endmodule


module controller(
  input  logic [6:0] op,
  input  logic [2:0] funct3,
  input  logic       funct7b5,
  input  logic       Zero,
  output logic [1:0] ResultSrc,
  output logic       MemWrite,
  output logic       PCSrc, ALUSrc,
  output logic       RegWrite, Jump,
  output logic [1:0] ImmSrc,
  output logic [2:0] ALUControl
);

  logic [1:0] ALUOp;
  logic       Branch;

  maindec md(
    .op(op),
    .ResultSrc(ResultSrc),
    .MemWrite(MemWrite),
    .Branch(Branch),
    .ALUSrc(ALUSrc),
    .RegWrite(RegWrite),
    .Jump(Jump),
    .ImmSrc(ImmSrc),
    .ALUOp(ALUOp)
  );

  aludec ad(
    .opb5(op[5]),
    .funct3(funct3),
    .funct7b5(funct7b5),
    .ALUOp(ALUOp),
    .ALUControl(ALUControl)
  );

  assign PCSrc = Branch & Zero; // Branch condicional; Jump tratado no datapath (PCSrcFinal)
endmodule

module maindec(
  input  logic [6:0] op,
  output logic [1:0] ResultSrc,
  output logic       MemWrite,
  output logic       Branch, ALUSrc,
  output logic       RegWrite, Jump,
  output logic [1:0] ImmSrc,
  output logic [1:0] ALUOp
);
  logic [10:0] controls;

  
  // controls = { RegWrite, ImmSrc(2), ALUSrc, MemWrite, ResultSrc(2), Branch, ALUOp(2), Jump }
  // Bits:       10        9:8        7       6        5:4          3       2:1       0
  assign {RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUOp, Jump} = controls;

  always_comb
    case(op)
      //RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
      7'b0000011: controls = 11'b1_00_1_0_01_0_00_0; // lw     : I-type, write-back ReadData
      7'b0100011: controls = 11'b0_01_1_1_00_0_00_0; // sw     : S-type, store
      7'b0110011: controls = 11'b1_00_0_0_00_0_10_0; // R-type : ALU operations (ALUOp=10)
      7'b1100011: controls = 11'b0_10_0_0_00_1_01_0; // beq    : B-type, subtract compare (ALUOp=01)
      7'b0010011: controls = 11'b1_00_1_0_00_0_10_0; // addi   : I-type ALU (ALUOp=10)
      7'b1101111: controls = 11'b1_11_0_0_10_0_00_1; // jal    : J-type, ResultSrc=10 (PC+4), Jump=1
      default:    controls = 11'b0_00_0_0_00_0_00_0; // AJUSTE: default seguro (sem X para não poluir)
    endcase
endmodule


module aludec(
  input  logic       opb5,
  input  logic [2:0] funct3,
  input  logic       funct7b5, 
  input  logic [1:0] ALUOp,
  output logic [2:0] ALUControl
);
  logic RtypeSub;
  assign RtypeSub = funct7b5 & opb5;  // TRUE para sub (R-type com bit 30 = 1)

  always_comb
    case(ALUOp)
      2'b00: ALUControl = 3'b000; // add (lw, sw, jal)
      2'b01: ALUControl = 3'b001; // sub (branch)
      default: case(funct3)       // R-type / I-type
                 3'b000: ALUControl = RtypeSub ? 3'b001 : 3'b000; // sub ou add/addi
                 3'b010: ALUControl = 3'b101; // slt/slti
                 3'b110: ALUControl = 3'b011; // or/ori
                 3'b111: ALUControl = 3'b010; // and/andi
                 default: ALUControl = 3'b000; // fallback para add
               endcase
    endcase
endmodule


module datapath(
  input  logic        clk, reset,
  input  logic [1:0]  ResultSrc, 
  input  logic        PCSrc, ALUSrc,
  input  logic        RegWrite, Jump,
  input  logic [1:0]  ImmSrc,
  input  logic [2:0]  ALUControl,
  output logic        Zero,
  output logic [31:0] PC,
  input  logic [31:0] Instr,
  output logic [31:0] ALUResult, WriteData,
  input  logic [31:0] ReadData
);

  logic [31:0] PCNext, PCPlus4, PCTarget;
  logic [31:0] ImmExt;
  logic [31:0] SrcA, SrcB;
  logic [31:0] Result;
  logic        PCSrcFinal;

  // PC register
  flopr #(32) pcreg(clk, reset, PCNext, PC); 

  // PC + 4
  adder pcadd4(PC, 32'd4, PCPlus4);

  // Branch / Jump target = PC + ImmExt
  adder pcaddbranch(PC, ImmExt, PCTarget);

  // PCSrcFinal combina Branch condicional (PCSrc) + Jump incondicional
  assign PCSrcFinal = PCSrc | Jump;

  mux2 #(32) pcmux(PCPlus4, PCTarget, PCSrcFinal, PCNext);
 
  // Banco de registradores
  regfile rf(
    .clk(clk),
    .we3(RegWrite),
    .a1(Instr[19:15]),
    .a2(Instr[24:20]),
    .a3(Instr[11:7]),
    .wd3(Result),
    .rd1(SrcA),
    .rd2(WriteData)
  ); 

  // Imediato
  extend ext(Instr[31:7], ImmSrc, ImmExt);

  // Fonte B da ALU
  mux2 #(32) srcbmux(WriteData, ImmExt, ALUSrc, SrcB);

  // ALU
  alu alu(
    .a(SrcA),
    .b(SrcB),
    .alucontrol(ALUControl),
    .result(ALUResult),
    .zero(Zero)
  );

  // Seleção do resultado final (write-back):
  // 00 -> ALUResult
  // 01 -> ReadData (lw)
  // 10 -> PCPlus4 (jal)
  mux3 #(32) resultmux(ALUResult, ReadData, PCPlus4, ResultSrc, Result);
endmodule


module regfile(
  input  logic        clk, 
  input  logic        we3, 
  input  logic [ 4:0] a1, a2, a3, 
  input  logic [31:0] wd3, 
  output logic [31:0] rd1, rd2
);
  logic [31:0] rf[31:0];

  always_ff @(posedge clk)
    if (we3 && a3 != 5'd0)//impedir escrita em x0
      rf[a3] <= wd3;	

  assign rd1 = (a1 != 0) ? rf[a1] : 32'b0;
  assign rd2 = (a2 != 0) ? rf[a2] : 32'b0;
endmodule

module adder(
  input  [31:0] a, b,
  output [31:0] y
);
  assign y = a + b;
endmodule


module extend(
  input  logic [31:7] instr,
  input  logic [1:0]  immsrc,
  output logic [31:0] immext
);
  always_comb begin
    logic [31:0] immI, immS, immB, immJ;

    immI = {{20{instr[31]}}, instr[31:20]};                                // I-type
    immS = {{20{instr[31]}}, instr[31:25], instr[11:7]};                    // S-type
    immB = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};    // B-type
    immJ = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};  // J-type

    case (immsrc)
      2'b00: immext = immI;
      2'b01: immext = immS;
      2'b10: immext = immB;
      2'b11: immext = immJ;
      default: immext = 32'hXXXX_XXXX;
    endcase
  end
endmodule


module flopr #(parameter WIDTH = 8)(
  input  logic             clk, reset,
  input  logic [WIDTH-1:0] d, 
  output logic [WIDTH-1:0] q
);
  always_ff @(posedge clk, posedge reset)
    if (reset) q <= '0;
    else       q <= d;
endmodule


module mux2 #(parameter WIDTH = 8)(
  input  logic [WIDTH-1:0] d0, d1, 
  input  logic             s, 
  output logic [WIDTH-1:0] y
);
  assign y = s ? d1 : d0; 
endmodule


module mux3 #(parameter WIDTH = 8)(
  input  logic [WIDTH-1:0] d0, d1, d2,
  input  logic [1:0]       s, 
  output logic [WIDTH-1:0] y
);
  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module imem(
  input  logic [31:0] a,
  output logic [31:0] rd
);
  logic [31:0] RAM[0:63]; 

  initial
    $readmemh("riscvtest.txt", RAM); 

  assign rd = RAM[a[31:2]]; // word aligned
endmodule


module dmem(
  input  logic        clk, we,
  input  logic [31:0] a, wd,
  output logic [31:0] rd
);
  logic [31:0] RAM[0:63];

  assign rd = RAM[a[31:2]]; // leitura combinacional

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule


module alu(
  input  logic [31:0] a, b,
  input  logic [2:0]  alucontrol,
  output logic [31:0] result,
  output logic        zero
);
  logic [31:0] condinvb, sum;
  logic        v;
  logic        isAddSub;

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum      = a + condinvb + alucontrol[0];
  assign isAddSub = (~alucontrol[2] & ~alucontrol[1]) |
                    (~alucontrol[1] &  alucontrol[0]);

  always_comb
    case (alucontrol) 
      3'b000:  result = sum;         // add
      3'b001:  result = sum;         // subtract
      3'b010:  result = a & b;       // and
      3'b011:  result = a | b;       // or
      3'b100:  result = a ^ b;       // xor
      3'b101:  result = sum[31] ^ v; // slt (signed)
      3'b110:  result = a << b[4:0]; // sll
      3'b111:  result = a >> b[4:0]; // srl (lógico)
      default: result = 32'hXXXX_XXXX;
    endcase

  assign zero = (result == 32'b0);
  assign v    = ~(alucontrol[0] ^ a[31] ^ b[31]) &
    (a[31] ^ sum[31]) & isAddSub; // overflow add/sub //
endmodule

