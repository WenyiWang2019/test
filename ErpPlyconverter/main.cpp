#include<stdio.h>
#include "PCCPointSet.h"
#include "erp.h"
using namespace pcc;
using namespace erp;
int main(int argc, char *argv[]) {

	CERPParameter para;
	para.Parse(argc, argv);

	CErp erp(para);
	if (para.mode == 0)
	{
		erp.ErpToPly(para);
	}
	else if (para.mode == 1)
	{
		erp.PlyToErp(para);
	}
	else
	{
		exit(1);
	
	}
		return 0;
}
