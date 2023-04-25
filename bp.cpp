/* 046267 Computer Architecture - Winter 20/21 - HW #1                  */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <cmath>
#include <vector>

using std::vector;
enum State {
    SNT, WNT, WT, ST
};

#define ADDRESS_SIZE (32-2)
#define BIMODAL_SIZE (2)

//Class for the FSM
class Table {
private:
    State predict;
public:

    Table(State predict) : predict(predict) {};

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
    vector<uint32_t> LocalHistories;

    vector<Table *> FSMs;


    int HT = -1;
    SIM_stats status;

    //return value -  The theoretical size of the TBT
    uint32_t calcTheoreticalSize() {
        uint32_t size = btbSize * (tagSize + ADDRESS_SIZE); //todo: check if 32 or 30
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
    }

public:
    BP(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
       bool isGlobalHist, bool isGlobalTable, int Shared) :
            btbSize(btbSize), historySize(historySize), tagSize(tagSize), fsmState(static_cast<State>(fsmState)),
            isHistGlobal(isHistGlobal), isTableGlobal(isTableGlobal), isShare(isShare) {

        for (int i = 0; i < btbSize; ++i) {
            Tags.push_back(0);
            Targets.push_back(0);
        }

        //History init
        if (isHistGlobal) {
            LocalHistories.push_back(0);
        } else {
            for (int i = 0; i < btbSize; ++i) {
                LocalHistories.push_back(0);
            }
        }

        //FSM init
        if (isTableGlobal) {
            FSMs.push_back(new Table(this->fsmState));
        } else {
            for (int i = 0; i < historySize; ++i) {
                FSMs.push_back(new Table(this->fsmState));
            }
        }

        HT = int(isGlobalHist) * 10 + int(isTableGlobal);

        //Init Status
        status.size = calcTheoreticalSize();
        status.br_num = 0;
        status.flush_num = 0;

    };

/***
 * Predict - this function predicts if branch is taken or not taken
 * @param pc - the branch instruction's pointer.
 * @param dst - the destination for the predicted action.
 *                 pc+4 if NT and target if taken.
 * @return prediction - false if NOT TAKEN, true if TAKEN
 */
    bool Predict(uint32_t pc, uint32_t *dst) {

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

    }

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
    void GetStatus(SIM_stats *curStats) { *curStats = status; }

};

/****** GIVEN FUNCTIONS ******/

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
            bool isGlobalHist, bool isGlobalTable, int Shared) {
    return -1;
}

bool BP_predict(uint32_t pc, uint32_t *dst) {
    return false;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
    return;
}

void BP_GetStats(SIM_stats *curStats) {
    return;
}


