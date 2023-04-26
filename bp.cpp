/* 046267 Computer Architecture - Winter 20/21 - HW #1                  */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <cmath>
#include <vector>
#include <map>
#include <iostream>

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

    void setPredict(State pred, bool isTaken) {
        predict = pred;
        UpdatePredict(isTaken);
    }

    void UpdatePredict(bool isTaken) {
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
    vector<vector<FSM *>> LocalTables;
    // Global
    uint32_t GlobalHistory;
    vector<FSM *> GlobalTable;
    int HT = -1;
    SIM_stats status;

    //return value -  The theoretical size of the TBT
    uint32_t calcSize();

public:
    BP() = default;

    void init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
              bool isGlobalHist, bool isGlobalTable, int Shared);

    BP(BP const &) = delete; // disable copy ctor
    void operator=(BP const &) = delete; // disable = operator
    ~BP() = default;

    static BP &getInstance() // make singleton
    {
        static BP instance; // Guaranteed to be destroyed.
        // Instantiated on first use.
        return instance;
    }

    int getTag(uint32_t pc);

    int getIndex(uint32_t pc);

    bool Predict(uint32_t pc, uint32_t *dst);

    void fillStatus(SIM_stats *curStats);

    void erase();

    void Update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst);

    void CropHistory(unsigned index);
};


/**********************************************
 * BP private methods implementation
 **********************************************/


uint32_t BP::calcSize() {
    uint32_t size = btbSize * (1+tagSize + ADDRESS_SIZE);
    uint32_t globalHistSize = historySize;
    uint32_t localHistSize = btbSize * historySize;
    uint32_t globalTblSize = pow(2, historySize) * BIMODAL_SIZE;
    uint32_t localTblSize = btbSize * pow(2, historySize) * BIMODAL_SIZE;
    switch (HT) {
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
              bool isGlobalHist, bool isGlobalTable, int Shared) {
    this->btbSize = btbSize;
    this->historySize = historySize;
    this->tagSize = tagSize;
    this->fsmState = static_cast<State>(fsmState);
    this->isHistGlobal = isGlobalHist;
    this->isTableGlobal = isGlobalTable;
    this->isShare = Shared;

    //Tags and Target init
    for (unsigned i = 0; i < btbSize; ++i) {
        Tags.push_back(0);
        Targets.push_back(0);
    }

    //History init
    if (isHistGlobal) {
        GlobalHistory = 0;
    } else {
        for (unsigned i = 0; i < btbSize; ++i) {
            LocalHistories.push_back(0);
        }
    }

    //Table init
    if (isTableGlobal) {
        for (unsigned i = 0; i < pow(2, historySize); ++i) {
            GlobalTable.push_back(new FSM(this->fsmState));
        }
    }

    else {
        for (unsigned i = 0; i < btbSize; ++i) {
            vector<FSM * > v1;
            for (int j = 0; j < pow(2, historySize); ++j) {
                v1.push_back(new FSM(this->fsmState));
                //LocalTables[i].push_back((new FSM(this->fsmState)));
            }
            LocalTables.push_back(v1);
        }
    }


    HT = int(isGlobalHist) * 10 + int(isTableGlobal);
    //Init Status
    status.size = calcSize();
    status.br_num = 0;
    status.flush_num = 0;

};


void BP::erase() {
    if (!LocalTables.empty()) {
        for (vector<FSM *> table : LocalTables) {
            if (!table.empty()) {
                for (FSM *FSM : table) {
                    delete FSM;
                }
            }
        }
    }
    if (!GlobalTable.empty()) {
        for (FSM *FSM: GlobalTable) {
            delete FSM;
        }
    }
}


int BP::getTag(uint32_t pc) {
    int shift = log2((double) (btbSize)) + 2;
    unsigned mask = ((1 << tagSize) - 1) << shift;
    return (pc & mask) >> shift;
}


int BP::getIndex(uint32_t pc) {
    int bitSizeOfBTB = log2((double) (btbSize));
    unsigned mask = ((1 << bitSizeOfBTB) - 1) << 2;
    return (pc & mask) >> 2;
}


bool BP::Predict(uint32_t pc, uint32_t *dst) {
    unsigned tag = getTag(pc);
    int index = getIndex(pc);
    if (Tags[index] != tag) {
        *dst = pc + 4;
        return false;
    }
    if (isHistGlobal) {
        if (isTableGlobal) {
            if (GlobalTable[GlobalHistory]->getPredict() == WNT ||
                GlobalTable[GlobalHistory]->getPredict() == SNT) {
                *dst = pc + 4;
                return false;
            } else {
                *dst = Targets[index];
                return true;
            }
        } else {
            if (LocalTables[index][GlobalHistory]->getPredict() == WNT ||
                LocalTables[index][GlobalHistory]->getPredict() == SNT) {
                *dst = pc + 4;
                return false;
            } else {
                *dst = Targets[index];
                return true;
            }
        }
    } else {
        if (isTableGlobal) {
            if (GlobalTable[LocalHistories[index]]->getPredict() == WNT ||
                GlobalTable[LocalHistories[index]]->getPredict() == SNT) {
                *dst = pc + 4;
                return false;
            } else {
                *dst = Targets[index];
                return true;
            }
        } else {
            if (LocalTables[index][LocalHistories[index]]->getPredict() == WNT ||
                LocalTables[index][LocalHistories[index]]->getPredict() == SNT) {
                *dst = pc + 4;
                return false;
            } else {
                *dst = Targets[index];
                return true;
            }
        }
    }
}


void BP::CropHistory(unsigned index){
    uint32_t mask;
    if(!isHistGlobal) {
        mask = ((1 << historySize) - 1);
        LocalHistories[index] &= mask;
    }
    else{
        mask = ((1 << historySize) - 1);
        GlobalHistory &= mask;
    }
}

void BP::Update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
    status.br_num++;
    uint32_t dst;
    Predict(pc, &dst);
    unsigned tag = getTag(pc);
    int index = getIndex(pc);

    if ((!taken && pred_dst != pc + 4) || (taken && pred_dst != targetPc)) status.flush_num++;



    bool isOverride = (Tags[index] != tag);
    if (isOverride) Tags[index] = tag;
    if (isOverride || dst != Targets[index]) Targets[index] = targetPc;


    switch (HT) {
        //both local
        case 0:
            isOverride ? LocalHistories[index] = taken : LocalHistories[index] <<= 1;
            LocalHistories[index] |= uint32_t (taken);
            CropHistory(index);
            isOverride ? LocalTables[LocalHistories[index]][index]->setPredict(fsmState, taken)
                       : LocalTables[LocalHistories[index]][index]->UpdatePredict(taken);
            break;
            //Hist local Table global
        case 1:
            isOverride ? LocalHistories[index] = taken : LocalHistories[index] <<= 1;
            LocalHistories[index] |= uint32_t (taken);
            CropHistory(index);
            isOverride ? GlobalTable[LocalHistories[index]]->setPredict(fsmState, taken)
                       : GlobalTable[LocalHistories[index]]->UpdatePredict(taken);
            break;
            //Hist Global Table local
        case 10:
            GlobalHistory <<= 1;
            GlobalHistory |= uint32_t (taken);
            CropHistory(0);
            isOverride ? LocalTables[index][GlobalHistory]->setPredict(fsmState, taken)
                       : LocalTables[index][GlobalHistory]->UpdatePredict(taken);
            break;
            //both global
        case 11:
            GlobalHistory <<= 1;
            GlobalHistory |= uint32_t (taken);
            CropHistory(0);
            GlobalTable[GlobalHistory]->UpdatePredict(taken);
            break;
    }

}


/***
 * GetStatus - returns the Simulator's status in curStats
 * @param curStats - pointer to return the Simulator's stat.
 */

void BP::fillStatus(SIM_stats *curStats) {
    (*curStats).size = status.size;
    (*curStats).br_num = status.br_num;
    (*curStats).flush_num = status.flush_num;
    return;
}


/**********************************************
 * Main Functions
 **********************************************/

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
            bool isGlobalHist, bool isGlobalTable, int Shared) {
    BP &bp = BP::getInstance();
    if (&bp == nullptr) {
        return -1;
    }
    bp.init(btbSize, historySize, tagSize, fsmState, isGlobalHist, isGlobalTable, Shared);
    return 0;
}


bool BP_predict(uint32_t pc, uint32_t *dst) {
    BP &bp = BP::getInstance();
    return bp.Predict(pc, dst);
}


void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
    BP &bp = BP::getInstance();
    bp.Update(pc, targetPc, taken, pred_dst);
}

void BP_GetStats(SIM_stats *curStats) {
    BP &bp = BP::getInstance();
    bp.fillStatus(curStats);
    bp.erase();
}