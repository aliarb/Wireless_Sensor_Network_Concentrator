# invoke SourceDir generated makefile for rfExamples.pem3
rfExamples.pem3: .libraries,rfExamples.pem3
.libraries,rfExamples.pem3: package/cfg/rfExamples_pem3.xdl
	$(MAKE) -f C:\Users\arabal\workspace_v7\zrfWsnConcentrator_CC2650_LAUNCHXL_TI_Synch/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\arabal\workspace_v7\zrfWsnConcentrator_CC2650_LAUNCHXL_TI_Synch/src/makefile.libs clean

