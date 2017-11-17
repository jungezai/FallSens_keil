@ECHO Start

@ECHO %1

C:\Users\huangxu\Desktop\hex2bin_2.2_XiaZaiBa\hex2bin.exe %1

@ECHO HEX2BIN %1 ....



@ECHO %~dp1

@ECHO %~p1

@ECHO %~dpn1

@ECHO > %~dpn1.txt

"C:\Program Files (x86)\Nordic Semiconductor\nRFgo Studio\nrfutil.exe" dfu genpkg %~dpn1.zip --application %~dpn1.bin --application-version 0xffffffff --dev-revision 0xffff --dev-type 0xffff --sd-req 0x0064


