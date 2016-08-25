# AS-IIS-VR-AR-DLL

## 概觀

- 本專案為DLL動態程式庫的原始碼，以Visual Studio 2013為開發環境
- 此專案需配合Unity專案：[AS-IIS-VR-AR-Unity](https://github.com/lctseng/AS-IIS-VR-AR-Unity)
- 資料夾結構(僅列出必要的資料夾)
  - AS-IIS-VR-AR-DLL
    - lib
    - Debug
    - Release
  - AS-IIS-VR-AR-Unity
    - Assets
    - Build (此為輸出資料夾，Unity建置時應把RoomFusion.exe放在此目錄內)
    - ProjectSettings
  - dependencies
    - eigen
      - Eigen (本資料夾內應包含Array、Core等檔案)
    - lz4
      - include (本資料夾內應包含lz4.h等檔案)
      - lib (本資料夾內應包含lz4.lib等檔案)
    - opencv
      - include
        - opencv (本資料夾內應包含cv.hpp等檔案)
        - opencv2 (本資料夾內應包含core.hpp等檔案)
      - x64
        - vc12
          - lib (本資料夾內應包含opencv_world310等檔案)
    - pthread
      - include (本資料夾內應包含pthread.h等檔案)
      - lib (本資料夾內應包含pthreadVC2.lib等檔案)
- dependencies資料夾內容可以在[這裡下載](https://drive.google.com/file/d/0B6KmSTnbOf-CYkxaSy1oSzN1Z28/view?usp=sharing)

## 編譯

### 基本環境

- 請以Visual Studio 2013開啟，若版本不對可能會造成無法正確編譯或執行
- 打開方案後，記得切換到Release模式後，再行編譯
- 方案請用 **x64** 來建置，切勿使用x86(Win32)。可透過組態管理員來調整

### 專案設定

- 本方案中的幾乎每個專案都有相依許多第三方程式庫，因此需要設置好他們的路徑
- 相依的第三方清單
  - ZED SDK
    - 請確定環境變數`ZED_INCLUDE_DIRS`的值是`C:\Program Files (x86)\ZED SDK\include`等類似的路徑
    - 請確定環境變數`ZED_LIBRARY_DIR`的值是`C:\Program Files (x86)\ZED SDK\lib`等類似的路徑
    - 請確定環境變數`ZED_LIBRARIES_64`的值是`sl_zed64.lib`或類似的結果
  - opencv
    - 在`dependencies`資料夾中
  - CUDA 7.5
    - 請安裝在系統中，確保Visual Studio能夠正確偵測到
  - eigen
    - 在`dependencies`資料夾中
  - pthread
    - 在`dependencies`資料夾中
  - lz4
    - 在`dependencies`資料夾中
- 由於程式庫路徑會隨著不同電腦而改變，請務必修改以下的程式庫路徑，以符合當下的環境
- 專案 -> 組態屬性 -> C/C++ -> 一般 -> 其他Include目錄

```text
$(SolutionDir)..\dependencies\opencv\include;$(SolutionDir)..\dependencies\opencv\include\opencv;$(SolutionDir)..\dependencies\lz4\include;$(SolutionDir)..\dependencies\eigen;$(SolutionDir)..\dependencies\pthread\include;$(ZED_INCLUDE_DIRS);%(AdditionalIncludeDirectories);$(CudaToolkitIncludeDir);
```

- 專案 -> 組態屬性 -> 連結器 -> 一般 -> 其他程式庫目錄

```text
$(SolutionDir)..\dependencies\lz4\lib;$(SolutionDir)..\dependencies\pthread\lib;$(SolutionDir)..\dependencies\opencv\x64\vc12\lib;$(ZED_LIBRARY_DIR);$(CudaToolkitLibDir);%(AdditionalLibraryDirectories);
```

- 專案 -> 組態屬性 -> 連結器 -> 輸入 -> 其他相依性

```text
kernel32.lib;user32.lib;gdi32.lib;winspool.lib;shell32.lib;ole32.lib;oleaut32.lib;uuid.lib;comdlg32.lib;advapi32.lib;glu32.lib;opencv_world310.lib;comctl32.lib;setupapi.lib;ws2_32.lib;vfw32.lib;cudart_static.lib;Mswsock.lib;pthreadVC2.lib;lz4.lib;$(ZED_LIBRARIES_64)
```

### 編譯成DLL

- 預設上直接編譯本專案就會生成.dll與對應的.lib
- 程式庫輸出目錄可以在 `專案 -> 組態屬性 -> 一般` 中調整。
- 預設的建置事件會把輸出的.dll與.lib複製到其他需要的地方
  - 專案 -> 組態屬性 -> 建置事件 -> 建置後事件 -> 命令列
  - 目前的設定會複製到四個資料夾底下，命令列為：

  ```text
  copy "$(SolutionDir)$(Platform)\$(Configuration)\$(AssemblyName).dll" "$(SolutionDir)..\AS-IIS-VR-AR-Unity\Build\RoomFusion_Data\Plugins\"
  copy "$(SolutionDir)$(Platform)\$(Configuration)\$(AssemblyName).dll" "$(SolutionDir)..\AS-IIS-VR-AR-Unity\Assets\Plugins\dll_windows\"
  copy "$(SolutionDir)$(Platform)\$(Configuration)\$(AssemblyName).lib" "$(SolutionDir)lib\"
  ```

  - 請注意，上述的建置步驟請確保每個檔案都有成功複製(一定是3個檔案，而不是只複製1個或2個)
  - 若因為.dll正在被其他程式使用(例如Unity)，請將程式關掉後，自行手動複製
  - 其中第一與第二行是最重要的，這把.dll複製到Unity的執行目錄與專案目錄下。同時也是最容易發生複製失敗的地方

## 程式碼架構

- 以下將對本DLL專案中各份原始碼做簡單的介紹
- `dllmain.cpp`：.dll的程式進入點，此檔案不需修改，保留預設內容就好
- `RoomFusionDLL.cpp/.h`：實做那些可以被外部呼叫的函數
  - 函數名稱以`rf_`開頭
  - `RoomFusionDLL.h`中明確定義了哪些函數要匯出給外部使用
  - 內部使用到的大多數全域變數是定義在`RoomFusionInternal.cpp`中
- `RoomFusionDLL.cu/.cuh`：實做與CUDA相關的程式碼
  - 任何需要呼叫到CUDA功能的程式碼都定義在這裡
  - 其中以`run`開頭的函數為CPU上的wrapper，真正GPU執行的為以`gpu`開頭的函數
- `RoomFusionInternal.cpp/.h`：定義了大多數內部使用的函數
  - 主要定義了那些不給外部呼叫的函數
  - 除了網路功能以外的內部函數都定義在這裡
  - 定義了大多數的內部使用全域變數
- `RoomFusionSocket.cpp/.h`：定義了網路連接相關的部分
  - 同樣是內部使用的函數，但特別將網路功能相關的部分獨立出來
  - 主要實做為Winsock與pthread
- `stdafx.cpp/.h`：Visual Studio產生的預設檔，不需理會
- `targetver.h`：Visual Studio產生的預設檔，不需理會

