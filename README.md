# NYCU Computer Architecture (2025 Fall)

本專案包含國立陽明交通大學資訊工程系 2025 秋季班「計算機結構 (Computer Architecture)」課程的完整實驗實作。

本課程主要探討現代微處理器的設計原理，從基礎的運算單元、管線化設計、超純量架構、亂序執行到記憶體階層設計。所有實作皆使用 Verilog 硬體描述語言進行開發，並通過嚴謹的單元測試與效能評估。

## 開發環境與工具

* **語言**: Verilog HDL
* **模擬工具**: Icarus Verilog (iverilog) / Synopsys VCS
* **波形觀察**: GTKWave
* **建置系統**: Makefile / Python scripts

## 實驗內容詳細說明

### Lab 1: Integer Multiply/Divide Unit (整數乘除法器)
本實驗目標為設計並實作一個基於 Val/Rdy 介面協定的迭代式整數乘法與除法單元，將兩個 32 位元運算元處理後產生 64 位元結果。

* **實作重點**:
    * **迭代式乘法器 (Iterative Multiplier)**: 採用移位與相加 (Shift-and-Add) 演算法，並分離資料路徑 (Datapath) 與控制單元 (Control Unit)。
    * **迭代式除法器 (Iterative Divider)**: 實作不回復餘數除法 (Non-restoring division) 或類似演算法。
    * **Booth 乘法器**: 實作 Radix-4 Booth 演算法以加速乘法運算，減少部分積 (Partial products) 的數量。
    * **介面協定**: 嚴格遵守 Latency-insensitive 的 Val/Rdy 握手協定，確保模組間的正確溝通。

### Lab 2: Pipelined RISC-V Processor (管線化 RISC-V 處理器)
本實驗將基礎的單週期處理器擴展為五級管線化 (5-stage Pipeline) 架構，並解決隨之而來的資料危障 (Data Hazards) 與控制危障 (Control Hazards)。

* **實作重點**:
    * **ISA 支援**: 完整實作 RV32I 指令集，並擴充支援部分 RV32M (乘除法) 指令。
    * **CSR 支援**: 加入 CSRW 指令以支援測試與診斷功能。
    * **管線化控制**: 實作 IF, ID, X, M, W 五個階段的控制邏輯。
    * **資料前饋 (Bypassing/Forwarding)**: 設計旁路網路以解決 Read-After-Write (RAW) 危障，減少管線停頓 (Stall) 次數並提升 IPC (Instructions Per Cycle)。
    * **效能評估**: 比較實作旁路前後的處理器效能差異。

### Lab 3: Superscalar RISC-V Processor (超純量 RISC-V 處理器)
本實驗目標為設計一個雙路 (Two-wide) 的循序執行 (In-order) 超純量處理器，使其能在單一週期內發射並執行兩道指令。

* **實作重點**:
    * **雙指令提取 (Dual Fetch)**: 修改提取級以支援同時從記憶體讀取兩道指令。
    * **指令分派 (Instruction Steering)**: 實作邏輯以判斷指令相依性，決定指令應由主管線 (Pipe A) 或次管線 (Pipe B) 執行。
    * **計分板 (Scoreboard)**: 設計計分板機制以集中管理暫存器的相依性，解決資料危障並確保寫入順序的正確性。
    * **結構性危障處理**: 處理當兩道指令需要使用相同硬體資源時的衝突。

### Lab 4: Out-of-Order RISC-V Processor - Reorder Buffer (亂序執行處理器與重排序緩衝區)
本實驗將處理器升級為支援亂序執行 (Out-of-Order Execution) 的架構，核心在於實作重排序緩衝區 (Reorder Buffer, ROB) 以確保指令能依序提交 (In-order Commit)。

* **實作重點**:
    * **重排序緩衝區 (ROB)**: 設計 ROB 以追蹤所有 inflight 指令的狀態，並儲存推測執行的結果。
    * **亂序執行與循序提交**: 允許指令在運算單元空閒時提前執行，但嚴格依照程式順序寫回暫存器檔案與記憶體，以維持精確例外 (Precise Exceptions)。
    * **暫存器更名 (Register Renaming)**: (若架構包含) 解決 WAW 與 WAR 危障。
    * **分支預測與回復**: 處理分支預測失敗時的管線清除與 ROB 狀態回復機制。

### Lab 5: Cache (快取記憶體)
本實驗為處理器加入指令快取 (I-Cache) 與資料快取 (D-Cache)，以降低記憶體存取延遲並提升整體系統效能。

* **實作重點**:
    * **基礎快取設計 (Baseline Design)**: 實作 Direct-mapped 或 Set-associative 快取。
    * **快取控制器 (Cache Controller)**: 設計複雜的有限狀態機 (FSM) 來處理 Hit/Miss 判斷、Refill 以及 Eviction 流程。
    * **寫入策略**: 實作 Write-back 或 Write-through 策略，並處理 Write-allocate。
    * **Victim Cache**: 加入 Victim Cache 以降低衝突失誤 (Conflict Misses)，提升快取命中率。
    * **效能分析**: 比較有無快取以及不同快取配置 (如關聯度、區塊大小) 對處理器執行時間的影響。

## 專案結構

```text
NYCU-Computer-Architecture/
├── Lab1/           # Integer Multiply/Divide Unit
├── Lab2/           # Pipelined RISC-V Processor
├── Lab3/           # Superscalar RISC-V Processor
├── Lab4/           # Out-of-Order RISC-V Processor (ROB)
├── Lab5/           # Cache Implementation
└── CA Lecture notes/ # Course Lecture Notes