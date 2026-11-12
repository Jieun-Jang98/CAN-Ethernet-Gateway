param(
    [string]$Port
)

$ErrorActionPreference = 'Stop'
$availablePorts = @([System.IO.Ports.SerialPort]::GetPortNames())
if (-not $Port) {
    if ($availablePorts.Count -ne 1) {
        Write-Error "COM 포트를 지정하세요. 사용 가능: $($availablePorts -join ', ')"
        exit 1
    }
    $Port = $availablePorts[0]
}

$serial = [System.IO.Ports.SerialPort]::new(
    $Port, 230400, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One
)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.ReadTimeout = 1000

try {
    $serial.Open()
    Write-Host "CAN 로그 연결: $Port / 230400 8N1 (종료: Ctrl+C)"
    while ($true) {
        try {
            Write-Host $serial.ReadLine().TrimEnd("`r")
        }
        catch [System.TimeoutException] {
            # The MCU may be reset or waiting for its first CAN period.
        }
    }
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}
