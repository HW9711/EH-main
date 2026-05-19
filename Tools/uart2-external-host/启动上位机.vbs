Option Explicit

Dim shell
Dim fso
Dim rootDir
Dim nodeCheck
Dim tempDir
Dim logFile

Set shell = CreateObject("WScript.Shell")
Set fso = CreateObject("Scripting.FileSystemObject")

rootDir = fso.GetParentFolderName(WScript.ScriptFullName)
tempDir = shell.ExpandEnvironmentStrings("%TEMP%")
logFile = tempDir & "\uart2-external-host-launch.log"

' Check Node.js first so double-click failures have visible feedback.
nodeCheck = shell.Run("cmd /c where node >nul 2>nul", 0, True)
If nodeCheck <> 0 Then
  MsgBox "Node.js was not found. Please install Node.js first.", vbCritical, "ExternalComm Host"
  WScript.Quit nodeCheck
End If

' The Node launcher starts localhost and opens the browser with the window hidden.
shell.CurrentDirectory = rootDir
shell.Run "cmd /c node scripts\launch.mjs >> """ & logFile & """ 2>&1", 0, False
