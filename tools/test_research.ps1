param([Parameter(Mandatory=$true)][string]$Zig)
$ErrorActionPreference='Stop'
$repoRoot=Split-Path $PSScriptRoot -Parent
Push-Location $repoRoot
try {
    & $Zig cc -std=c11 -Wall -Wextra -I fsd_logic test/test_fsd_core.c fsd_logic/fsd_handler.c fsd_logic/fsd_profile.c -o test/test_fsd_core.exe
    if($LASTEXITCODE -ne 0){throw 'shared core compilation failed'}
    & ./test/test_fsd_core.exe
    if($LASTEXITCODE -ne 0){throw 'shared core tests failed'}
    & $Zig c++ -std=c++17 -Wall -Wextra -I fsd_logic test/test_esp32_core.cpp esp32/.firmware/fsd_handler.cpp -o test/test_esp32_core.exe
    if($LASTEXITCODE -ne 0){throw 'ESP32 core compilation failed'}
    & ./test/test_esp32_core.exe
    if($LASTEXITCODE -ne 0){throw 'ESP32 core tests failed'}
    & $Zig c++ -std=c++17 -Wall -Wextra test/test_research.cpp -o test/test_research.exe
    if($LASTEXITCODE -ne 0){throw 'research compilation failed'}
    & ./test/test_research.exe
    if($LASTEXITCODE -ne 0){throw 'research tests failed'}
    & $Zig c++ -std=c++17 -Wall -Wextra tools/research_replay.cpp -o test/research_replay.exe
    if($LASTEXITCODE -ne 0){throw 'replay compilation failed'}
    Get-Content examples/research.replay | & ./test/research_replay.exe 14
    if($LASTEXITCODE -ne 0){throw 'V14 replay failed'}
    Get-Content examples/research.replay | & ./test/research_replay.exe 13
    if($LASTEXITCODE -ne 0){throw 'V13 replay failed'}
    $rejected=& $Zig cc -x c -fsyntax-only -DRESEARCH_PHYSICAL_TX=1 esp32/.firmware/research_policy.h 2>&1
    if($LASTEXITCODE -eq 0 -or "$rejected" -notmatch 'Physical CAN TX is forbidden'){throw 'TX compile-time interlock failed'}
    Write-Output 'Physical TX override: correctly rejected at compile time'
} finally { Pop-Location }
