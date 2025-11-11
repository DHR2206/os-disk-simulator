// Disk Scheduling Simulator - C++ Version
// Converted from disk.py
//
// EDITED: Bug fixes applied.

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <map>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <getopt.h>
#include <iomanip>

using namespace std;

// Constants
const int MAXTRACKS = 1000;

// States that a request/disk go through
enum State {
    STATE_NULL = 0,
    STATE_SEEK = 1,
    STATE_ROTATE = 2,
    STATE_XFER = 3,
    STATE_DONE = 4
};

// Structure to hold block information
struct BlockInfo {
    int track;
    double angle;
    int name;
    BlockInfo(int t, double a, int n) : track(t), angle(a), name(n) {}
};

// Structure to hold request
struct Request {
    int block;
    int index;
    Request(int b, int i) : block(b), index(i) {}
};

// Disk class
class Disk {
private:
    // Configuration
    string addr;
    string addrDesc;
    string lateAddr;
    string lateAddrDesc;
    string policy;
    double seekSpeed;
    double rotateSpeed;
    int skew;
    int window;
    bool compute;
    bool graphics;
    string zoning;

    // Disk geometry
    vector<BlockInfo> blockInfoList;
    map<int, int> blockToTrackMap;
    map<int, double> blockToAngleMap;
    map<int, pair<int, int>> tracksBeginEnd;
    vector<int> blockAngleOffset;
    int maxBlock;

    // Track information
    map<int, double> tracks;
    double trackWidth;

    // Arm position and movement
    int armTrack;
    double armSpeedBase;
    double armSpeed;
    double armX1, armX2;
    double armTargetX1;
    int armTarget;

    // Request queue
    vector<Request> requestQueue;
    vector<State> requestState;
    int requestCount;
    int currentIndex;
    int currentBlock;

    // Late requests
    vector<int> requests;
    vector<int> lateRequests;
    int lateCount;

    // Scheduling window
    int currWindow;
    int fairWindow;

    // Simulation state
    State state;
    double angle;
    double timer;

    // Timing
    double seekBegin, rotBegin, xferBegin;
    double seekTotal, rotTotal, xferTotal;
    double totalEst;

    // Control
    bool isDone;

public:
    Disk(const string& addr, const string& addrDesc, const string& lateAddr,
         const string& lateAddrDesc, const string& policy, double seekSpeed,
         double rotateSpeed, int skew, int window, bool compute, bool graphics,
         const string& zoning);

    void Go();

private:
    void InitBlockLayout();
    vector<int> MakeRequests(const string& addr, const string& addrDesc);
    void PrintAddrDescMessage(const string& value);

    void GetNextIO();
    void Animate();
    void UpdateTime();
    void DoRequestStats();
    void PrintStats();

    pair<int, int> DoSATF(const vector<Request>& rList);
    vector<Request> DoSSTF(const vector<Request>& rList);
    void PlanSeek(int track);
    bool DoneWithSeek();
    bool DoneWithRotation();
    bool DoneWithTransfer();
    bool RadiallyCloseTo(double a1, double a2);

    void SwitchState(State newState);
    void AddRequest(int block);
    int GetWindow();
    void UpdateWindow();

    vector<string> Split(const string& s, char delimiter);
};

// Helper function to split strings
vector<string> Disk::Split(const string& s, char delimiter) {
    vector<string> tokens;
    stringstream ss(s);
    string token;
    while (getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// Constructor
Disk::Disk(const string& addr, const string& addrDesc, const string& lateAddr,
           const string& lateAddrDesc, const string& policy, double seekSpeed,
           double rotateSpeed, int skew, int window, bool compute, bool graphics,
           const string& zoning)
    : addr(addr), addrDesc(addrDesc), lateAddr(lateAddr), lateAddrDesc(lateAddrDesc),
      policy(policy), seekSpeed(seekSpeed), rotateSpeed(rotateSpeed), skew(skew),
      window(window), compute(compute), graphics(graphics), zoning(zoning) {

    // Track info
    trackWidth = 40;
    tracks[0] = 140;
    tracks[1] = tracks[0] - trackWidth;
    tracks[2] = tracks[1] - trackWidth;

    if (seekSpeed > 1 && ((int)trackWidth % (int)seekSpeed != 0)) {
        cerr << "Seek speed (" << seekSpeed << ") must divide evenly into track width (" << trackWidth << ")" << endl;
        exit(1);
    }
    
    // Initialize block layout
    InitBlockLayout();

    // Make requests
    this->requests = MakeRequests(addr, addrDesc);
    this->lateRequests = MakeRequests(lateAddr, lateAddrDesc);

    // Fairness window
    if (this->policy == "BSATF" && this->window != -1) {
        fairWindow = this->window;
    } else {
        fairWindow = -1;
    }

    cout << "REQUESTS ";
    for (size_t i = 0; i < this->requests.size(); i++) {
        cout << this->requests[i];
        if (i < this->requests.size() - 1) cout << ",";
    }
    cout << endl << endl;

    if (!this->lateRequests.empty()) {
        cout << "LATE REQUESTS ";
        for (size_t i = 0; i < this->lateRequests.size(); i++) {
            cout << this->lateRequests[i];
            if (i < this->lateRequests.size() - 1) cout << ",";
        }
        cout << endl << endl;
    }

    if (!this->compute) {
        cout << endl;
        cout << "For the requests above, compute the seek, rotate, and transfer times." << endl;
        cout << "Use -c to see the answers." << endl;
        cout << endl;
    }


    // Arm initialization
    armTrack = 0;
    armSpeedBase = seekSpeed;
    armSpeed = seekSpeed;
    
    // BUG FIX 1: The arm's X position must be initialized to the
    // center of the starting track (Track 0), not 0.
    armX1 = tracks[armTrack] - (trackWidth / 2.0);
    armX2 = armX1 + trackWidth;


    // Request queue initialization
    requestCount = 0;
    for (size_t i = 0; i < this->requests.size(); i++) {
        requestQueue.push_back(Request(this->requests[i], i));
        requestState.push_back(STATE_NULL);
    }

    // Scheduling window
    currWindow = this->window;

    // Initial state
    currentIndex = -1;
    currentBlock = -1;
    state = STATE_NULL;

    // Angle and timer
    angle = 0.0;
    timer = 0;

    // Stats
    seekTotal = 0.0;
    rotTotal = 0.0;
    xferTotal = 0.0;

    // Late requests
    lateCount = 0;

    // Control
    isDone = false;
}

void Disk::InitBlockLayout() {
    vector<string> zones = Split(zoning, ',');
    if (zones.size() != 3) {
        cerr << "Zoning must have exactly 3 values" << endl;
        exit(1);
    }

    for (size_t i = 0; i < zones.size(); i++) {
        cout << "z " << i << " " << zones[i] << endl;
        blockAngleOffset.push_back(stoi(zones[i]) / 2);
    }

    // Outer track (track 0)
    int track = 0;
    int angleOffset = 2 * blockAngleOffset[track];
    int block = 0;
    for (int angle = 0; angle < 360; angle += angleOffset) {
        block = angle / angleOffset;
        cout << track << " " << angleOffset << " " << block << endl;
        blockToTrackMap[block] = track;
        blockToAngleMap[block] = angle;
        blockInfoList.push_back(BlockInfo(track, angle, block));
    }
    tracksBeginEnd[track] = make_pair(0, block);
    int pblock = block + 1;

    // Middle track (track 1)
    track = 1;
    int skewVal = this->skew;
    angleOffset = 2 * blockAngleOffset[track];
    for (int angle = 0; angle < 360; angle += angleOffset) {
        block = (angle / angleOffset) + pblock;
        cout << track << " " << skewVal << " " << angleOffset << " " << block << endl;
        blockToTrackMap[block] = track;
        blockToAngleMap[block] = angle + (angleOffset * skewVal);
        blockInfoList.push_back(BlockInfo(track, angle + (angleOffset * skewVal), block));
    }
    tracksBeginEnd[track] = make_pair(pblock, block);
    pblock = block + 1;

    // Inner track (track 2)
    track = 2;
    skewVal = 2 * this->skew;
    angleOffset = 2 * blockAngleOffset[track];
    for (int angle = 0; angle < 360; angle += angleOffset) {
        block = (angle / angleOffset) + pblock;
        cout << track << " " << skewVal << " " << angleOffset << " " << block << endl;
        blockToTrackMap[block] = track;
        blockToAngleMap[block] = angle + (angleOffset * skewVal);
        blockInfoList.push_back(BlockInfo(track, angle + (angleOffset * skewVal), block));
    }
    tracksBeginEnd[track] = make_pair(pblock, block);
    
    // BUG FIX 3: maxBlock should be the *last* block created,
    // not the *starting* block of the last track.
    maxBlock = block; 

    // Adjust angles
    for (auto& pair : blockToAngleMap) {
        pair.second = fmod(pair.second + 180, 360);
    }
}

vector<int> Disk::MakeRequests(const string& addr, const string& addrDesc) {
    if (addr == "-1") {
        vector<string> desc = Split(addrDesc, ',');
        if (desc.size() != 3) {
            PrintAddrDescMessage(addrDesc);
            return vector<int>();
        }

        int numRequests = stoi(desc[0]);
        int maxRequest = stoi(desc[1]);
        int minRequest = stoi(desc[2]);

        if (maxRequest == -1) {
            // This now uses the corrected maxBlock value
            maxRequest = maxBlock;
        }

        vector<int> tmpList;
        for (int i = 0; i < numRequests; i++) {
            tmpList.push_back((rand() % (maxRequest - minRequest + 1)) + minRequest);
        }
        return tmpList;
    } else {
        vector<string> addrList = Split(addr, ',');
        vector<int> result;
        for (const string& s : addrList) {
            result.push_back(stoi(s));
        }
        return result;
    }
}

void Disk::PrintAddrDescMessage(const string& value) {
    cerr << "Bad address description (" << value << ")" << endl;
    cerr << "The address description must be a comma-separated list of length three, without spaces." << endl;
    cerr << "For example, \"10,100,0\" would indicate that 10 addresses should be generated, with" << endl;
    cerr << "100 as the maximum value, and 0 as the minimum. A max of -1 means just use the highest" << endl;
    cerr << "possible value as the max address to generate." << endl;
    exit(1);
}

void Disk::SwitchState(State newState) {
    state = newState;
    requestState[currentIndex] = newState;
}

bool Disk::RadiallyCloseTo(double a1, double a2) {
    // Handle wrap-around: the shortest distance between two angle
    double v = abs(a1 - a2);

    if (v > 180.0) {

        v = 360.0 - v;

    }
    // Ensure targetAngle is positive
    if (targetAngle < 0) targetAngle += 360.0;
    // Be a bit more tolerant for float comparison
    return v < (rotateSpeed + 0.0001);
}

bool Disk::DoneWithTransfer() {
    int angleOffset = blockAngleOffset[armTrack];
    double targetAngle = fmod(blockToAngleMap[currentBlock] + angleOffset, 360);
    if (RadiallyCloseTo(angle, targetAngle)) {
        SwitchState(STATE_DONE);
        requestCount++;
        
        return true;
    }
    return false;
}

bool Disk::DoneWithRotation() {
    int angleOffset = blockAngleOffset[armTrack];
    double targetAngle = fmod(blockToAngleMap[currentBlock] - angleOffset, 360);
    // Ensure targetAngle is positive (fmod can return negative values)

    if (targetAngle < 0) targetAngle += 360.0;
    if (RadiallyCloseTo(angle, targetAngle)) {
        SwitchState(STATE_XFER);
        return true;
    }
    return false;
}

void Disk::PlanSeek(int track) {
    seekBegin = timer;
    SwitchState(STATE_SEEK);
    if (track == armTrack) {
        rotBegin = timer;
        SwitchState(STATE_ROTATE);
        return;
    }
    armTarget = track;
    armTargetX1 = tracks[track] - (trackWidth / 2.0);
    if (track >= armTrack) {
        armSpeed = armSpeedBase;
    } else {
        armSpeed = -armSpeedBase;
    }
}

bool Disk::DoneWithSeek() {
    armX1 += armSpeed;
    armX2 += armSpeed;

    if ((armSpeed > 0.0 && armX1 >= armTargetX1) || (armSpeed < 0.0 && armX1 <= armTargetX1)) {
        armTrack = armTarget;

        // BUG FIX 2: "Snap" the arm to the exact target position upon arrival.
        // This ensures the *next* seek calculation starts from the correct place.
        armX1 = armTargetX1;
        armX2 = armX1 + trackWidth;
        
        return true;
    }
    return false;
}

pair<int, int> Disk::DoSATF(const vector<Request>& rList) {
    int minBlock = -1;
    int minIndex = -1;
    double minEst = -1;

    for (const Request& req : rList) {
        if (requestState[req.index] == STATE_DONE) {
            continue;
        }

        int track = blockToTrackMap[req.block];
        double angle = blockToAngleMap[req.block];

        // Estimate seek time
        int dist = abs(armTrack - track);
        
        // This uses the arm's *current* position (armX1) vs. the target's
        double seekEst = abs((tracks[track] - (trackWidth / 2.0)) - armX1) / armSpeedBase;

        // Estimate rotate time
        int angleOffset = blockAngleOffset[track];
        double angleAtArrival = fmod(this->angle + (seekEst * rotateSpeed), 360);

        double rotDist = (angle - angleOffset) - angleAtArrival;
        while (rotDist < 0.0) rotDist += 360.0; // Ensure positive rotation
        rotDist = fmod(rotDist, 360.0); // Handle full wraps
        
        double rotEst = rotDist / rotateSpeed;

        // Transfer time
        double xferEst = (angleOffset * 2.0) / rotateSpeed;

        double totalEst = seekEst + rotEst + xferEst;

        if (minEst == -1 || totalEst < minEst) {
            minEst = totalEst;
            minBlock = req.block;
            minIndex = req.index;
        }
    }

    this->totalEst = minEst;
    return make_pair(minBlock, minIndex);
}

vector<Request> Disk::DoSSTF(const vector<Request>& rList) {
    int minDist = -1; // Use -1 to handle first case
    vector<Request> trackList;

    for (const Request& req : rList) {
        if (requestState[req.index] == STATE_DONE) {
            continue;
        }

        int track = blockToTrackMap[req.block];
        int dist = abs(armTrack - track);

        if (minDist == -1 || dist < minDist) {
            trackList.clear();
            trackList.push_back(req);
            minDist = dist;
            } else if (dist == minDist) { // FIX: Compare to minDist, not character 'O'
            trackList.push_back(req);
        }
    }

    return trackList;
}

void Disk::UpdateWindow() {
    if (fairWindow == -1 && currWindow > 0 && currWindow < (int)requestQueue.size()) {
        currWindow++;
    }
}

int Disk::GetWindow() {
    if (currWindow <= -1) {
        return requestQueue.size();
    } else {
        if (fairWindow != -1) {
            if (requestCount > 0 && (requestCount % fairWindow == 0)) {
                currWindow = currWindow + fairWindow;
            }
            return currWindow;
        } else {
            return currWindow;
        }
    }
}

void Disk::AddRequest(int block) {
    requestQueue.push_back(Request(block, requestQueue.size()));
    requestState.push_back(STATE_NULL);
}

void Disk::GetNextIO() {
    // Check if done
    if (requestCount == (int)requestQueue.size()) {
        UpdateTime();
        PrintStats();
        isDone = true;
        return;
    }

    // Apply policy
    if (policy == "FIFO") {
        currentBlock = requestQueue[requestCount].block;
        currentIndex = requestQueue[requestCount].index;
        vector<Request> singleReq;
        singleReq.push_back(requestQueue[requestCount]);
        DoSATF(singleReq);
    } else if (policy == "SATF" || policy == "BSATF") {
        int endIndex = GetWindow();
        if (endIndex > (int)requestQueue.size()) {
            endIndex = requestQueue.size();
        }
        vector<Request> subQueue(requestQueue.begin(), requestQueue.begin() + endIndex);
        pair<int, int> result = DoSATF(subQueue);
        currentBlock = result.first;
        currentIndex = result.second;
    } else if (policy == "SSTF") {
        int endIndex = GetWindow();
        if (endIndex > (int)requestQueue.size()) {
            endIndex = requestQueue.size();
        }
        vector<Request> subQueue(requestQueue.begin(), requestQueue.begin() + endIndex);
        vector<Request> trackList = DoSSTF(subQueue);
        pair<int, int> result = DoSATF(trackList);
        currentBlock = result.first;
        currentIndex = result.second;
    } else {
        cerr << "Policy (" << policy << ") not implemented" << endl;
        exit(1);
    }

    // Do the seek
    PlanSeek(blockToTrackMap[currentBlock]);

    // Add late request
    if (!lateRequests.empty() && lateCount < (int)lateRequests.size()) {
        AddRequest(lateRequests[lateCount]);
        lateCount++;
    }
}

void Disk::UpdateTime() {
    // In console mode, just track the values
}

void Disk::Animate() {
    // Increment timer
    timer++;

    // Rotate disk
    angle += rotateSpeed;
    if (angle >= 360.0) {
        angle -= 360.0; // Use subtraction for precision
    }

    // Process current state
    if (state == STATE_SEEK) {
        if (DoneWithSeek()) {
            rotBegin = timer;
            SwitchState(STATE_ROTATE);
        }
    }
    if (state == STATE_ROTATE) {
        if (DoneWithRotation()) {
            xferBegin = timer;
            SwitchState(STATE_XFER);
        }
    }
    if (state == STATE_XFER) {
        if (DoneWithTransfer()) {
            DoRequestStats();
            SwitchState(STATE_DONE);
            UpdateWindow();
            int prevBlock = currentBlock;
            GetNextIO();
            if (!isDone) {
                int nextBlock = currentBlock;
                if (blockToTrackMap[prevBlock] == blockToTrackMap[nextBlock]) {
                    auto& trackRange = tracksBeginEnd[armTrack];
                    if ((prevBlock == trackRange.second && nextBlock == trackRange.first) ||
                        (prevBlock + 1 == nextBlock)) {
                        rotBegin = timer;
                        seekBegin = timer;
                        xferBegin = timer;
                        SwitchState(STATE_XFER);
                    }
                }
            }
        }
    }
}

void Disk::DoRequestStats() {
    double seekTime = rotBegin - seekBegin;
    double rotTime = xferBegin - rotBegin;
    double xferTime = timer - xferBegin;
    double totalTime = timer - seekBegin;

    if (compute) {
        cout << "Block: " << setw(3) << currentBlock
             << "  Seek:" << setw(3) << (int)seekTime
             << "  Rotate:" << setw(3) << (int)rotTime
             << "  Transfer:" << setw(3) << (int)xferTime
             << "  Total:" << setw(4) << (int)totalTime << endl;
    }

    seekTotal += seekTime;
    rotTotal += rotTime;
    xferTotal += xferTime;
}

void Disk::PrintStats() {
    if (compute) {
        cout << endl << "TOTALS      Seek:" << setw(3) << (int)seekTotal
             << "  Rotate:" << setw(3) << (int)rotTotal
             << "  Transfer:" << setw(3) << (int)xferTotal
             << "  Total:" << setw(4) << (int)timer << endl << endl;
    }
}

void Disk::Go() {
    GetNextIO();
    while (!isDone) {
        Animate();
    }
}

// Main function
int main(int argc, char* argv[]) {
    // Default options
    int seed = 0;
    string addr = "-1";
    string addrDesc = "5,-1,0";
    string seekSpeed = "1";
    string rotSpeed = "1";
    string policy = "FIFO";
    int window = -1;
    int skewOffset = 0;
    string zoning = "30,30,30";
    bool graphics = false;
    string lateAddr = "-1";
    string lateAddrDesc = "0,-1,0";
    bool compute = false;

    // Parse command-line options
    struct option long_options[] = {
        {"seed",         required_argument, 0, 's'},
        {"addr",         required_argument, 0, 'a'},
        {"addrDesc",     required_argument, 0, 'A'},
        {"seekSpeed",    required_argument, 0, 'S'},
        {"rotSpeed",     required_argument, 0, 'R'},
        {"policy",       required_argument, 0, 'p'},
        {"schedWindow",  required_argument, 0, 'w'},
        {"skewOffset",   required_argument, 0, 'o'},
        {"zoning",       required_argument, 0, 'z'},
        {"graphics",     no_argument,       0, 'G'},
        {"lateAddr",     required_argument, 0, 'l'},
        {"lateAddrDesc", required_argument, 0, 'L'},
        {"compute",      no_argument,       0, 'c'},
        {0, 0, 0, 0}
    };

    int opt;
    int option_index = 0;
    while ((opt = getopt_long(argc, argv, "s:a:A:S:R:p:w:o:z:Gl:L:c", long_options, &option_index)) != -1) {
        switch (opt) {
            case 's': seed = atoi(optarg); break;
            case 'a': addr = optarg; break;
            case 'A': addrDesc = optarg; break;
            case 'S': seekSpeed = optarg; break;
            case 'R': rotSpeed = optarg; break;
            case 'p': policy = optarg; break;
            case 'w': window = atoi(optarg); break;
            case 'o': skewOffset = atoi(optarg); break;
            case 'z': zoning = optarg; break;
            case 'G': graphics = true; break;
            case 'l': lateAddr = optarg; break;
            case 'L': lateAddrDesc = optarg; break;
            case 'c': compute = true; break;
            default:
                cerr << "Usage: " << argv[0] << " [options]" << endl;
                return 1;
        }
    }

    // Set random seed
    srand(seed);

    cout << "OPTIONS seed " << seed << endl;
    cout << "OPTIONS addr " << addr << endl;
    cout << "OPTIONS addrDesc " << addrDesc << endl;
    cout << "OPTIONS seekSpeed " << seekSpeed << endl;
    cout << "OPTIONS rotateSpeed " << rotSpeed << endl;
    cout << "OPTIONS skew " << skewOffset << endl;
    cout << "OPTIONS window " << window << endl;
    cout << "OPTIONS policy " << policy << endl;
    cout << "OPTIONS compute " << (compute ? "true" : "false") << endl;
    cout << "OPTIONS graphics " << (graphics ? "true" : "false") << endl;
    cout << "OPTIONS zoning " << zoning << endl;
    cout << "OPTIONS lateAddr " << lateAddr << endl;
    cout << "OPTIONS lateAddrDesc " << lateAddrDesc << endl;
    cout << endl;

    if (window == 0) {
        cerr << "Scheduling window (" << window << ") must be positive or -1 (which means a full window)" << endl;
        return 1;
    }

    if (graphics && !compute) {
        cout << "\nWARNING: Graphics mode not supported in C++ version (console only)\n" << endl;
        cout << "Setting compute flag to True\n" << endl;
        compute = true;
    }

    // Create disk simulator
    Disk d(addr, addrDesc, lateAddr, lateAddrDesc, policy,
           stod(seekSpeed), stod(rotSpeed), skewOffset, window,
           compute, false, zoning);

    // Run simulation
    d.Go();

    return 0;
}
