///////////////////////////////////////////////////////////////////////
//
// Preconditions: ujoint.sldasm exists in the specified folder and
//                contains a component named bracket-1.
//      
//
// Postconditions: None
//
// NOTE: Scroll down see the code for the stdafx.h header file.
//
///////////////////////////////////////////////////////////////////////
#include "stdafx.h"

#define ERROR_SIMULATION_PLUGIN_LOAD_FAILURE  1;
#define ERROR_SOLIDWORKS_APP_LOAD_FAILURE     2;
#define EOK                                   0;
#define ERROR_SIMULATION_CALLBACK_FAILURE     3;
#define ERROR_OPEN_PART_FAILURE               4;
using namespace std;


VARIANT_BOOL retVal = VARIANT_FALSE;
HRESULT hres = NOERROR;

void OpenPart(ISldWorks* swApp, IModelDoc2** swModel);
void OpenPartSilent(ISldWorks* swApp, IModelDoc** swModel);

int main(int argc, char *argv[])
{
  double study_duration = 0.012;
  
  //Initialize COM
  CoInitialize(NULL);

  //Use ATL smart pointers
  CComPtr<ISldWorks> swApp;
  CComPtr<ICosmosWorks> cosmos_app;
  CComPtr<IModelDoc2> swModel;
  CComPtr<ICWStudyManager> study_manager;
  CComPtr<IComponent2> swComponent;
  CComPtr<ICWModelDoc> cwModel;
  CComPtr<ICWStudy> study;
  CComPtr<ICWNonLinearStudyOptions> non_linear_study_options;
  CComPtr<ICWLoadsAndRestraintsManager> restraints_manager;
  CComPtr<ICWLoadsAndRestraints> restraint;

    try {
      //Open or attach to the currently running instance of the sldworks COM server
      hres = swApp.CoCreateInstance(__uuidof(SldWorks), NULL, CLSCTX_LOCAL_SERVER);
      if (hres != S_OK) {
        printf("ERROR: Failed to load SolidWorks app");
        throw ERROR_SOLIDWORKS_APP_LOAD_FAILURE;
      }

      //Open part
      OpenPart(swApp, &swModel);

      // Load Simulation plugin
      struct IDispatch *sim_dispatch;
      CComBSTR sim(L"SldWorks.Simulation");
      CComBSTR sim_addin(L"C:\\Program Files\\SolidWorks Corp\\SolidWorks\\Simulation\\cosworks.dll");
      long ret_val = EOK;
      swApp->LoadAddIn(sim_addin, &ret_val);
      hres = swApp->GetAddInObject(sim, &sim_dispatch);
      if (hres != S_OK) {
        printf("Failed to load Simulation");
        throw ERROR_SIMULATION_PLUGIN_LOAD_FAILURE;
      }
      // Handle Simulation dispatch and callback
      ICwAddincallback* cos;
      hres = sim_dispatch->QueryInterface(__uuidof(ICwAddincallback), reinterpret_cast<void**>(&cos));
      sim_dispatch->Release();
      hres = cos->get_CosmosWorks(&cosmos_app);
      if (hres != S_OK) {
        printf("Failed to attach the Simulation callback");
        throw ERROR_SIMULATION_CALLBACK_FAILURE;
      }
      // Get Simulation handle for model in active doc
      cosmos_app->get_ActiveDoc(&cwModel);

      // Get pre-created study
      cwModel->get_StudyManager(&study_manager);
      study_manager->GetStudy(0, &study);

      // Change study duration
      study->get_NonLinearStudyOptions(&non_linear_study_options);
      non_linear_study_options->put_EndTime(study_duration);

      study->get_LoadsAndRestraintsManager(&restraints_manager);
      long error = 0;
      long countin = 0;
      // in_out
      restraints_manager->get_Count(&countin);
      restraints_manager->GetLoadsAndRestraints(2, &error, &restraint);
      BSTR name =L"";
      restraint->get_Name(&name);
      restraint->get_Type(&countin);
      restraint->get_State(&countin);
      restraint->SuppressUnSuppress();
      restraint->get_State(&countin);
      //restraint->

      //restraint->
      struct IDispatch *restraint_dispatch;
      long sel = swSelSIMELEMENT;
      countin = 0;
      restraint->get_EntityCount(&countin);


      printf("count: %f", countin);
      restraint->GetEntityAt(0, &sel, &restraint_dispatch);
      ICWRestraint* rest;
      restraint_dispatch->QueryInterface(__uuidof(ICWRestraint), reinterpret_cast<void**>(&rest));
      restraint_dispatch->Release();
      rest->RestraintBeginEdit();

      //Close documents
      swApp->CloseAllDocuments(true, &retVal);
    }

    catch (int e) {
      //Release COM references
      swApp = NULL;
      swModel = NULL;
      swComponent = NULL;

      //Uninitialize COM
      CoUninitialize();
      return e;
    }

  //Release COM references
  swApp = NULL;
  swModel = NULL;
  swComponent = NULL;

  //Uninitialize COM
  CoUninitialize();
  return EOK;
}

void OpenPartSilent(ISldWorks* swApp, IModelDoc** swModel) {
  CComBSTR sFileName(L"C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\cad\\Rev8\\straight_wire.SLDPRT");
  CComBSTR sDefaultConfiguration(L"FEA");
  long fileerror, filewarning;
  IModelDoc* swModelPart;
  //hres = swApp->OpenDoc6(sFileName, swDocPART, swOpenDocOptions_Silent, sDefaultConfiguration, &fileerror, &filewarning, &swModelPart);
  hres = swApp->IOpenDocSilent(sFileName, swDocPART, &fileerror, &swModelPart);
  if (S_OK != hres || swModelPart == NULL) {
    throw ERROR_OPEN_PART_FAILURE; // error
  }
  *swModel = swModelPart;
}

void OpenPart(ISldWorks* swApp, IModelDoc2** swModel) {
  CComBSTR sFileName(L"C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\cad\\Rev8\\straight_wire.SLDPRT");
  CComBSTR sDefaultConfiguration(L"FEA");
  long fileerror, filewarning;
  IModelDoc2* swModelPart;
  hres = swApp->OpenDoc6(sFileName, swDocPART, swOpenDocOptions_Silent, sDefaultConfiguration, &fileerror, &filewarning, &swModelPart);
  //hres = swApp->IOpenDocSilent(sFileName, swDocPART, &fileerror, &swModelPart);
  if (S_OK != hres || swModelPart == NULL) {
    throw ERROR_OPEN_PART_FAILURE; // error
  }
  *swModel = swModelPart;
}