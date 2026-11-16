$S = $(if ($env:HAND_TEST_DIR) { $env:HAND_TEST_DIR } else { (Get-Location).Path })
$M = $(if ($env:HAND_MODELS_DIR) { $env:HAND_MODELS_DIR } else { "C:\Fran\repos\depthai-unity\unity_bridge\depthai_hand_tracker\models" })
$exe = "C:\FuBIntLab\repos\sai-unity\plugin\target\Release\main_depthai.exe"
function Run([string]$name, [bool]$hands, [hashtable]$envs, [int]$seconds) {
    foreach ($k in @("SAI_HAND_REPLAY","SAI_HAND_NO_DEPTH","SAI_IR_PROJECTOR","SAI_RECORD_DIR","SAI_USE_STEREO","SAI_USE_FEATURE_TRACKER","SAI_FAST_VIO","SAI_HAND_REPLAY_LOOP","SAI_USE_COLOR","SAI_RECORD_NO_COLOR","SAI_HAND_LOG","SAI_HAND_SEARCH_INTERVAL_MS")) { Remove-Item "Env:$k" -ErrorAction SilentlyContinue }
    $env:SAI_RUN_SECONDS = "$seconds"; $env:SAI_HAND_TRACKING_DEBUG = "1"
    foreach ($k in $envs.Keys) { Set-Item "Env:$k" $envs[$k] }
    $a = @()
    if ($hands) { $a = @("`"$M\palm_detection_sh4.blob`"", "`"$M\hand_landmark_full_sh4.blob`"") }
    if ($a.Count -gt 0) {
        $p = Start-Process -FilePath $exe -ArgumentList $a -RedirectStandardOutput "$S\th_$name.out" -RedirectStandardError "$S\th_$name.err" -PassThru -WindowStyle Hidden
    } else {
        $p = Start-Process -FilePath $exe -RedirectStandardOutput "$S\th_$name.out" -RedirectStandardError "$S\th_$name.err" -PassThru -WindowStyle Hidden
    }
    $null = $p.WaitForExit(($seconds + 60) * 1000)
    if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force }
}
Run "A_sai_only" $false @{ SAI_USE_COLOR = "0" } 240
Run "B_hands_throttled_idle" $true @{} 240
"done" | Out-File "$S\th_done.txt"
