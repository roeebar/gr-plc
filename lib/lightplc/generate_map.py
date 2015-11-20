#!/usr/bin/python
import sys

def grayToBinary( num ):
    mask = num >> 1;
    while mask != 0:
    	num = num ^ mask;
    	mask = mask >> 1;
    return num;
	
def generateIQMap (nBits):
	n = 1 << nBits;
	imap = [0 for x in range(n*n)] 
	qmap = [0 for x in range(n*n)] 
	j=0
	powerSum = 0;
	for q in range(0,1<<nBits):
		qValue = 2*((n+n/2-1-grayToBinary(q))%n)-(n-1);
		for i in range (0,1<<nBits):
			iValue = 2*((n+n/2-1-grayToBinary(i))%n)-(n-1);
			imap[j] = iValue;
			qmap[j] = qValue;
			powerSum = powerSum + iValue**2+qValue**2;
			j = j + 1
	return (imap,qmap, (powerSum/(n**2))**-0.5)

def generateIMap (nBits):
	n = 1 << nBits;
	imap = [0 for x in range(n)] 
	j=0
	powerSum = 0;
	for i in range(0,1<<nBits):
		iValue = 2*((n+n/2-1-grayToBinary(i))%n)-(n-1);
		imap[j] = iValue;
		powerSum = powerSum + iValue**2;
		j = j +1
	return (imap,powerSum)


modulations = [{"name":"BPSK", "bits":1},
               {"name":"QPSK", "bits":2},
               {"name":"QAM8", "bits":3},
               {"name":"QAM16", "bits":4},
               {"name":"QAM64", "bits":6},
               {"name":"QAM256", "bits":8},
               {"name":"QAM1024", "bits":10},
               {"name":"QAM4096", "bits":12},]
for m in modulations:
	sys.stdout.write("const unsigned int MAP_" + m["name"] + "_NBITS = " + str(m["bits"]) + ";\n")
	sys.stdout.write("const complex MAP_"+m["name"]+"["+str(1<<m["bits"])+"] = {\n")
	if m["name"] == "BPSK":
		imap,powerSum=generateIMap(m["bits"]);	
		scale = (powerSum/len(imap))**-0.5
		for j in range(len(imap)-1):
			sys.stdout.write ("complex("+str(imap[j])+", 0),\n");
		sys.stdout.write ("complex("+str(imap[-1])+", 0)\n};\n");
		
	elif m["name"] == "QAM8":
		imap,powerSum=generateIMap(m["bits"]-1)
		scale = (powerSum/len(imap) + 1.29**2)**-0.5
		for j in range(len(imap)):
			sys.stdout.write ("complex("+str(imap[j])+", -1.29),\n");
		for j in range(len(imap)-1):
			sys.stdout.write ("complex("+str(imap[j])+", 1.29),\n");
		sys.stdout.write ("complex("+str(imap[-1])+", 1.29)\n};\n");
	else:
		imap,qmap,scale=generateIQMap(m["bits"]/2);
		for j in range(len(imap)-1):
			sys.stdout.write ("complex("+str(imap[j])+", "+str(qmap[j])+"), // "+("0b{0:0"+str(m["bits"])+"b}").format(j)+"\n")
		sys.stdout.write ("complex("+str(imap[-1])+", "+str(qmap[-1])+") // "+("0b{0:0"+str(m["bits"])+"b}").format(j+1)+"\n};\n")

	sys.stdout.write ("const float MAP_"+m["name"]+"_SCALE = "+str(scale)+";\n\n")	
