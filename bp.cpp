/* 046267 Computer Architecture - Winter 20/21 - HW #1                  */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <cmath>
#include <vector>
#include <map>

using std::vector;
using std::map;

enum State {
    SNT, WNT, WT, ST
};

#define ADDRESS_SIZE (32-2)
#define BIMODAL_SIZE (2)


/***********************************************
 * class FSM
  **********************************************/


class FSM {
private:
    State predict;
public:

    FSM(State predict) : predict(predict) {};

    State getPredict() { return predict; }

    void setPredict(bool isTaken) {
        switch (predict) {
            case SNT:
                predict = isTaken ? WNT : SNT;
                break;
            case WNT:
                predict = isTaken ? WT : SNT;
                break;
            case WT:
                predict = isTaken ? ST : WNT;
                break;
            case ST:
                predict = isTaken ? ST : WT;
                break;
        }
    }
};


/**********************************************
 * class BP
 **********************************************/


class BP {
private:
    unsigned btbSize;
    unsigned historySize;
    unsigned tagSize;
    State fsmState;
    bool isHistGlobal;
    bool isTableGlobal;
    int isShare;
    vector<uint32_t> Tags;
    vector<uint32_t> Targets;
    // Local
    vector<uint32_t> LocalHistories;
    vector<vector<FSM*>> LocalTables;
    // Global
    uint32_t GlobalHistory;
    vector<FSM*> GlobalTable;
    int HT = -1;
    SIM_stats status;

    //return value -  The theoretical size of the TBT
    uint32_t calcTheoreticalSize();

public:
    BP() = default;
    void init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
       bool isGlobalHist, bool isGlobalTable, int Shared);
    BP(BP const&) = delete; // disable copy ctor
    void operator=(BP const&) = delete; // disable = operator
    ~BP();
    static BP& getInstance() // make singleton
    {
        static BP instance; // Guaranteed to be destroyed.
        // Instantiated on first use.
        return instance;
    }


/***
 * Predict - this function predicts if branch is taken or not taken
 * @param pc - the branch instruction's pointer.
 * @param dst - the destination for the predicted action.
 *                 pc+4 if NT and target if taken.
 * @return prediction - false if NOT TAKEN, true if TAKEN
 */
    bool Predict(uint32_t pc, uint32_t *dst);
};


/**********************************************
 * BP private methods implementation
 **********************************************/


uint32_t BP::calcTheoreticalSize()
{
    uint32_t size = btbSize * (tagSize + ADDRESS_SIZE);
    uint32_t globalHistSize = historySize;
    uint32_t localHistSize = btbSize * historySize;
    uint32_t globalTblSize = pow(2, historySize) * BIMODAL_SIZE;
    uint32_t localTblSize = btbSize * pow(2, historySize) * BIMODAL_SIZE;
    switch (HT)
    {
    //both local
    case 0:
    return size + localHistSize + localTblSize;

    //Hist local Table global
    case 1:
    return size + localHistSize + globalTblSize;

    //Hist Global Table local
    case 10:
    return size + globalHistSize + localTblSize;

    //both global
    case 11:
    return size + globalHistSize + globalTblSize;
    }
    return 0;
}



/**********************************************
 * BP public methods implementation
 **********************************************/


void BP::init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
       bool isGlobalHist, bool isGlobalTable, int Shared)
{
    this->btbSize = btbSize;
    this->historySize = historySize;
    this->tagSize= tagSize;
    this->fsmState = static_cast<State>(fsmState);
    this->isHistGlobal = isHistGlobal;
    this->isTableGlobal = isTableGlobal;
    this->isShare = isShare;
    //Tags and Target init
        for (int i = 0; i < btbSize; ++i)
        {
            Tags.push_back(0);
            Targets.push_back(0);
        }
        //History init
        if (isHistGlobal)
        {
            GlobalHistory = 0;
        }
        else
        {
            for (int i = 0; i < btbSize; ++i) {
                LocalHistories.push_back(0);
            }
        }
        //Table init
        if (isTableGlobal)
        {
            for (int i = 0; i < pow(2, historySize); ++i) {
                GlobalTable.push_back(new FSM(this->fsmState));
            }
        }
        else
        {
            for (int i = 0; i < btbSize; ++i)
            {
                for (int j = 0; j < pow(2, historySize); ++j)
                {
                    LocalTables[i].push_back(new FSM(this->fsmState));
                }
            }
        }
        HT = int(isGlobalHist) * 10 + int(isTableGlobal);
        //Init Status
        status.size = calcTheoreticalSize();
        status.br_num = 0;
        status.flush_num = 0;
};


BP::~BP()
{
    if (!LocalTables.empty())
    {
        for (vector<FSM*> table : LocalTables)
        {
            if (!table.empty())
            {
                for (FSM* FSM : table)
                {
                    delete FSM;
                }
            }
        }
    }
    if (!GlobalTable.empty())
    {
        for (FSM *FSM: GlobalTable) {
            delete FSM;
        }
    }
}


bool BP::Predict(uint32_t pc, uint32_t *dst)
{
    //TODO: add methods for each case and update status.br_num
    switch (HT) {
    //both local
    case 0:

    break;
    //Hist local Table global
    case 1:

    break;
    //Hist Global Table local
    case 10:

    break;
    //both global
    case 11:

    break;
    }
    return true;
}


/**********************************************
 * Main Functions
 **********************************************/


/***
 * Update - update predictor with actual decision
 * @param pc - the branch instruction's pointer.
 * @param targetPc - the branch instruction's pointer.
 */
    void Update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {

    }
/***
 * GetStatus - returns the Simulator's status in curStats
 * @param curStats - pointer to return the Simulator's stat.
 */
    void GetStatus(SIM_stats *curStats)
{
    //{ *curStats = status; }

};

/****** GIVEN FUNCTIONS ******/

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
            bool isGlobalHist, bool isGlobalTable, int Shared) {
    BP& bp = BP::getInstance();
    if (&bp)
    {
        return -1;
    }
    bp.init(btbSize, historySize, tagSize, fsmState, isGlobalHist, isGlobalTable, Shared);
    return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst)
{
    BP& bp = BP::getInstance();
    int mask = 1 >> bp
    int Instruction_tag = pc <<
    return false;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
    return;
}

void BP_GetStats(SIM_stats *curStats) {
    return;
}


