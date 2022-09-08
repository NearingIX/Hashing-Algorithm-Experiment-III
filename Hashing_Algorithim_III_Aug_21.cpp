//Hashing Algorithm Experiment III

#include <iostream>
#include <bitset>
#include <vector>
#include <algorithm>
#include <string>
#include <cmath>
#include <iterator>
#include <sstream>
#include <cstdio>
using namespace std;

class message {
public:
    string messageInput;
    string messageASCIIString;
    string padValueString;
    int messageASCIIInt;

public:
    message() {
    string messageInput;
    string messageASCIIString = "";
    string padValueString = "";
    int messageASCIIInt = 0;
    };

    string gatherMessage() {
    cout << "Input message to hash: ";
    getline (cin, messageInput);

    //Provide message if no input is given
    if (messageInput.size() == 0)
    {
        messageInput = "a";
    };

    cout << "You entered: " << messageInput + 
    "\nA message with an initial length of: " << messageInput.length() << "\n";
    return messageInput;
};

void convertToASCII() {
    for (unsigned int i = 0; i < messageInput.size(); i++) {
        messageASCIIInt += messageInput[i];
        messageASCIIString += to_string(messageInput[i]);
    }
    double long padValue = pow(messageASCIIInt, 64);
    padValue = pow(padValue, 4);
    padValueString = to_string(padValue);
    cout << "ASCII padValue: " << padValue << endl;
    cout << "ASCII String: " << messageASCIIString << endl;
};
};

class mainVector {
    public:
    vector<bitset<8> > messageDigest;
    vector<bitset<8> > finalMessageDigest;
    string padValueString;
    vector<bitset<8> > firstBlock;
    vector<bitset<8> > secondBlock;
    vector<bitset<8> > concatenateFirstBlock;
    vector<bitset<8> > concatenateSecondBlock;
    vector<bitset<8> > messageBlockOne;
    vector<bitset<8> > messageBlockTwo;
    vector<bitset<8> > messageBlockThree;
    vector<bitset<8> > messageBlockFour;
    stringstream sstream;
    string completeMessageDigest;
    stringstream messageStream;

    public:
        mainVector() {
        vector<bitset<8> > messageDigest;
        vector<bitset<8> > finalMessageDigest;
        string padValueString = "";
        vector<bitset<8> > firstBlock;
        vector<bitset<8> > secondBlock;
        vector<bitset<8> > concatenateFirstBlock;
        vector<bitset<8> > concatenateSecondBlock;
        vector<bitset<8> > messageBlockOne;
        vector<bitset<8> > messageBlockTwo;
        vector<bitset<8> > messageBlockThree;
        vector<bitset<8> > messageBlockFour;
        stringstream sstream;
        string completeMessageDigest;
        stringstream messageStream;
        }
    
void padmessageDigest() {
    do {
        bitset<8> vectorPad = 01010101;
        messageDigest.push_back(vectorPad);
    }
    while (messageDigest.size() < 512);
};

void printmessageDigest() {
 for (auto i: messageDigest) {
        cout << i << ' ';
    };
};

void reversemessageDigest() {
    reverse(messageDigest.begin(), messageDigest.end());
};

void binaryConverter(string conversionInput) {
    for (size_t i = 0; i < conversionInput.size(); i++) {
       bitset<8> binaryDigest = bitset<8>(conversionInput[i]);
       messageDigest.push_back(binaryDigest);
    }
    printmessageDigest();
};

void firstMutation(){
for (size_t index = 0; index < messageDigest.size(); ++ index) {
    //Every nth element in the vector is changed
    vector<bitset<8>>::iterator begin = messageDigest.begin() + index;
    if (index % 7 == 0) {
        *begin |= 1 << 6;
        messageDigest.push_back(*begin);
    };
    if (index % 13 == 0) {
        *begin |= 1 << 4;
        messageDigest.push_back(*begin);
    };
    if (index % 3 == 0) {
        *begin |= 1 << 6;
        messageDigest.push_back(*begin);
    };
};
};

void messageDigestSize() {
    cout << "messageDigest Size" << messageDigest.size() << endl;
};

//Resize messageDigest vector to 512 bytes
void messageDigestTrim() {
    messageDigest.resize(512);
};

//Create four equal 128 sized messageBlocks
void divideMessageDigest() {
firstBlock = vector<bitset<8> > (messageDigest.begin(), messageDigest.begin() + messageDigest.size() / 2);
secondBlock = vector<bitset<8> > (messageDigest.begin() + messageDigest.size() / 2, messageDigest.end());
//Divide firstBlock
messageBlockOne = vector<bitset<8> > (firstBlock.begin(), firstBlock.begin() + firstBlock.size() / 2);
messageBlockTwo = vector<bitset<8> > (firstBlock.begin() + firstBlock.size() / 2, firstBlock.end());
//Divide secondBlock
messageBlockThree = vector<bitset<8> > (secondBlock.begin(), secondBlock.begin() + secondBlock.size() / 2);
messageBlockFour = vector<bitset<8> > (secondBlock.begin() + secondBlock.size() / 2, secondBlock.end());
};

//firstBlock Mutation Series
void firstBlockMutationOne() {
    messageBlockOne[0] = messageBlockOne[29] ^ messageBlockOne[8];
    messageBlockOne[1] = messageBlockOne[13] | messageBlockOne[72];
    messageBlockOne[2] = messageBlockOne[68] & messageBlockOne[41];
    messageBlockOne[3] = messageBlockOne[83] ^ messageBlockOne[58];
    messageBlockOne[4] = messageBlockOne[121] & messageBlockOne[99];
    messageBlockOne[5] = messageBlockOne[76] & messageBlockOne[47];
    messageBlockOne[6] = messageBlockOne[55] & messageBlockOne[28];
    messageBlockOne[7] = messageBlockOne[116] ^ messageBlockOne[76];
    messageBlockOne[8] = messageBlockOne[4] | messageBlockOne[82];
    messageBlockOne[9] = messageBlockOne[110] ^ messageBlockOne[91];
    messageBlockOne[10] = messageBlockOne[47] ^ messageBlockOne[16];
    messageBlockOne[11] = messageBlockOne[98] & messageBlockOne[71];
    messageBlockOne[12] = messageBlockOne[54] | messageBlockOne[81];
    messageBlockOne[13] = messageBlockOne[113] & messageBlockOne[108];
    messageBlockOne[14] = messageBlockOne[28] ^ messageBlockOne[52];
    messageBlockOne[15] = messageBlockOne[67] | messageBlockOne[26];
    };

void firstBlockMutationTwo() {
    messageBlockOne[16] = messageBlockOne[82] ^ messageBlockOne[35];
    messageBlockOne[17] = messageBlockOne[119] & messageBlockOne[74];
    messageBlockOne[18] = messageBlockOne[120] ^ messageBlockOne[98];
    messageBlockOne[19] = messageBlockOne[3] | messageBlockOne[121];
    messageBlockOne[20] = messageBlockOne[60] ^ messageBlockOne[117];
    messageBlockOne[21] = messageBlockOne[36] | messageBlockOne[73];
    messageBlockOne[22] = messageBlockOne[99] & messageBlockOne[37];
    messageBlockOne[23] = messageBlockOne[100] ^ messageBlockOne[123];
    messageBlockOne[24] = messageBlockOne[109] ^ messageBlockOne[42];
    messageBlockOne[25] = messageBlockOne[111] & messageBlockOne[6];
    messageBlockOne[26] = messageBlockOne[56] ^ messageBlockOne[30];
    messageBlockOne[27] = messageBlockOne[11] ^ messageBlockOne[119];
    messageBlockOne[28] = messageBlockOne[117] | messageBlockOne[122];
    messageBlockOne[29] = messageBlockOne[122] | messageBlockOne[118];
    messageBlockOne[30] = messageBlockOne[84] & messageBlockOne[92];
    messageBlockOne[31] = messageBlockOne[48] & messageBlockOne[57];
};

void firstBlockMutationThree() {
    messageBlockOne[32] = messageBlockOne[2] | messageBlockOne[46];
    messageBlockOne[33] = messageBlockOne[27] | messageBlockOne[34];
    messageBlockOne[34] = messageBlockOne[118] & messageBlockOne[7];
    messageBlockOne[35] = messageBlockOne[75] | messageBlockOne[15];
    messageBlockOne[36] = messageBlockOne[19] | messageBlockOne[80];
    messageBlockOne[37] = messageBlockOne[42] ^ messageBlockOne[90];
    messageBlockOne[38] = messageBlockOne[115] & messageBlockOne[126];
    messageBlockOne[39] = messageBlockOne[57] | messageBlockOne[124];
    messageBlockOne[40] = messageBlockOne[35] ^ messageBlockOne[56];
    messageBlockOne[41] = messageBlockOne[10] ^ messageBlockOne[61];
    messageBlockOne[42] = messageBlockOne[53] ^ messageBlockOne[59];
    messageBlockOne[43] = messageBlockOne[41] | messageBlockOne[27];
    messageBlockOne[44] = messageBlockOne[59] & messageBlockOne[83];
    messageBlockOne[45] = messageBlockOne[9] & messageBlockOne[107];
    messageBlockOne[46] = messageBlockOne[81] ^ messageBlockOne[48];
    messageBlockOne[47] = messageBlockOne[97] & messageBlockOne[100];
    messageBlockOne[48] = messageBlockOne[32] ^ messageBlockOne[53];
};

void firstBlockMutationFour() {
    messageBlockOne[49] = messageBlockOne[26] | messageBlockOne[84];
    messageBlockOne[50] = messageBlockOne[46] | messageBlockOne[17];
    messageBlockOne[51] = messageBlockOne[101] ^ messageBlockOne[5];
    messageBlockOne[52] = messageBlockOne[85] ^ messageBlockOne[36];
    messageBlockOne[53] = messageBlockOne[74] | messageBlockOne[19];
    messageBlockOne[54] = messageBlockOne[49] | messageBlockOne[10];
    messageBlockOne[55] = messageBlockOne[31] | messageBlockOne[29];
    messageBlockOne[56] = messageBlockOne[108] & messageBlockOne[38];
    messageBlockOne[57] = messageBlockOne[112] | messageBlockOne[75];
    messageBlockOne[58] = messageBlockOne[30] ^ messageBlockOne[79];
    messageBlockOne[59] = messageBlockOne[5] ^ messageBlockOne[9];
    messageBlockOne[60] = messageBlockOne[124] & messageBlockOne[68];
    messageBlockOne[61] = messageBlockOne[77] & messageBlockOne[62];
    messageBlockOne[62] = messageBlockOne[58] & messageBlockOne[33];
    messageBlockOne[63] = messageBlockOne[12] ^ messageBlockOne[18];
    messageBlockOne[64] = messageBlockOne[8] & messageBlockOne[4];
    
};

void firstBlockMutationFive() {
    messageBlockOne[65] = messageBlockOne[25] | messageBlockOne[70];
    messageBlockOne[66] = messageBlockOne[86] & messageBlockOne[69];
    messageBlockOne[67] = messageBlockOne[52] & messageBlockOne[93];
    messageBlockOne[68] = messageBlockOne[96] | messageBlockOne[97];
    messageBlockOne[69] = messageBlockOne[40] ^ messageBlockOne[112];
    messageBlockOne[70] = messageBlockOne[16] & messageBlockOne[89];
    messageBlockOne[71] = messageBlockOne[91] ^ messageBlockOne[24];
    messageBlockOne[72] = messageBlockOne[80] & messageBlockOne[14];
    messageBlockOne[73] = messageBlockOne[65] | messageBlockOne[111];
    messageBlockOne[74] = messageBlockOne[102] & messageBlockOne[125];
    messageBlockOne[75] = messageBlockOne[20] ^ messageBlockOne[67];
    messageBlockOne[76] = messageBlockOne[107] | messageBlockOne[51];
    messageBlockOne[77] = messageBlockOne[33] ^ messageBlockOne[78];
    messageBlockOne[78] = messageBlockOne[95] & messageBlockOne[120];
    messageBlockOne[79] = messageBlockOne[123] ^ messageBlockOne[113];
    messageBlockOne[80] = messageBlockOne[22] ^ messageBlockOne[23];
    messageBlockOne[81] = messageBlockOne[114] | messageBlockOne[25];
};

void firstBlockMutationSix() {
    messageBlockOne[82] = messageBlockOne[37] | messageBlockOne[63];
    messageBlockOne[83] = messageBlockOne[50] & messageBlockOne[49];
    messageBlockOne[84] = messageBlockOne[87] ^ messageBlockOne[85];
    messageBlockOne[85] = messageBlockOne[103] | messageBlockOne[106];
    messageBlockOne[86] = messageBlockOne[43] | messageBlockOne[54];
    messageBlockOne[87] = messageBlockOne[92] ^ messageBlockOne[20];
    messageBlockOne[88] = messageBlockOne[73] & messageBlockOne[11];
    messageBlockOne[89] = messageBlockOne[69] & messageBlockOne[77];
    messageBlockOne[90] = messageBlockOne[51] ^ messageBlockOne[109];
    messageBlockOne[91] = messageBlockOne[17] | messageBlockOne[94];
    messageBlockOne[92] = messageBlockOne[21] | messageBlockOne[2];
    messageBlockOne[93] = messageBlockOne[61] ^ messageBlockOne[43];
    messageBlockOne[94] = messageBlockOne[94] & messageBlockOne[101];
    messageBlockOne[95] = messageBlockOne[34] ^ messageBlockOne[86];
    messageBlockOne[96] = messageBlockOne[6] ^ messageBlockOne[66];
    messageBlockOne[97] = messageBlockOne[18] | messageBlockOne[32]; 
};

void firstBlockMutationSeven() {
    messageBlockOne[98] = messageBlockOne[24] ^ messageBlockOne[105];
    messageBlockOne[99] = messageBlockOne[88] | messageBlockOne[115];
    messageBlockOne[100] = messageBlockOne[63] | messageBlockOne[128];
    messageBlockOne[101] = messageBlockOne[104] ^ messageBlockOne[114];
    messageBlockOne[102] = messageBlockOne[128] & messageBlockOne[87];
    messageBlockOne[103] = messageBlockOne[72] & messageBlockOne[13];
    messageBlockOne[104] = messageBlockOne[7] ^ messageBlockOne[95];
    messageBlockOne[105] = messageBlockOne[127] | messageBlockOne[60];
    messageBlockOne[106] = messageBlockOne[126] ^ messageBlockOne[3];
    messageBlockOne[107] = messageBlockOne[70] | messageBlockOne[102];
    messageBlockOne[108] = messageBlockOne[23] ^ messageBlockOne[45];
    messageBlockOne[109] = messageBlockOne[44] & messageBlockOne[39];
    messageBlockOne[110] = messageBlockOne[38] ^ messageBlockOne[44];
    messageBlockOne[111] = messageBlockOne[78] & messageBlockOne[65];
    messageBlockOne[112] = messageBlockOne[125] ^ messageBlockOne[104];
    messageBlockOne[113] = messageBlockOne[64] & messageBlockOne[127];
    messageBlockOne[114] = messageBlockOne[89] | messageBlockOne[116];
};

void firstBlockMutationEight() {
    messageBlockOne[115] = messageBlockOne[14] | messageBlockOne[110];
    messageBlockOne[116] = messageBlockOne[62] ^ messageBlockOne[55];
    messageBlockOne[117] = messageBlockOne[106] ^ messageBlockOne[50];
    messageBlockOne[118] = messageBlockOne[1] ^ messageBlockOne[21];
    messageBlockOne[119] = messageBlockOne[90] & messageBlockOne[103];
    messageBlockOne[120] = messageBlockOne[105] & messageBlockOne[88];
    messageBlockOne[121] = messageBlockOne[93] ^ messageBlockOne[1];
    messageBlockOne[122] = messageBlockOne[79] & messageBlockOne[22];
    messageBlockOne[123] = messageBlockOne[45] ^ messageBlockOne[96];
    messageBlockOne[124] = messageBlockOne[71] ^ messageBlockOne[64];
    messageBlockOne[125] = messageBlockOne[66] | messageBlockOne[40];
    messageBlockOne[126] = messageBlockOne[39] | messageBlockOne[31];
    messageBlockOne[127] = messageBlockOne[15] | messageBlockOne[12];
};

//secondBlock Mutation Series
void secondBlockMutationOne() {
    messageBlockTwo[0] = messageBlockTwo[14] | messageBlockTwo[14];
    messageBlockTwo[1] = messageBlockTwo[23] | messageBlockTwo[27];
    messageBlockTwo[2] = messageBlockTwo[58] ^ messageBlockTwo[33];
    messageBlockTwo[3] = messageBlockTwo[89] & messageBlockTwo[40];
    messageBlockTwo[4] = messageBlockTwo[56] ^ messageBlockTwo[55];
    messageBlockTwo[5] = messageBlockTwo[61] & messageBlockTwo[116];
    messageBlockTwo[6] = messageBlockTwo[69] & messageBlockTwo[128];
    messageBlockTwo[7] = messageBlockTwo[74] & messageBlockTwo[49];
    messageBlockTwo[8] = messageBlockTwo[108] & messageBlockTwo[26];
    messageBlockTwo[9] = messageBlockTwo[118] | messageBlockTwo[125];
    messageBlockTwo[10] = messageBlockTwo[119] | messageBlockTwo[32];
    messageBlockTwo[11] = messageBlockTwo[38] & messageBlockTwo[89];
    messageBlockTwo[12] = messageBlockTwo[107] & messageBlockTwo[95];
    messageBlockTwo[13] = messageBlockTwo[0] ^ messageBlockTwo[54];
    messageBlockTwo[14] = messageBlockTwo[106] | messageBlockTwo[4];
    messageBlockTwo[15] = messageBlockTwo[1] & messageBlockTwo[31];
};
    void secondBlockMutationTwo() {
    messageBlockTwo[16] = messageBlockTwo[60] | messageBlockTwo[127];
    messageBlockTwo[17] = messageBlockTwo[88] & messageBlockTwo[115];
    messageBlockTwo[18] = messageBlockTwo[109] | messageBlockTwo[61];
    messageBlockTwo[19] = messageBlockTwo[22] ^ messageBlockTwo[13];
    messageBlockTwo[20] = messageBlockTwo[75] | messageBlockTwo[81];
    messageBlockTwo[21] = messageBlockTwo[57] ^ messageBlockTwo[66];
    messageBlockTwo[22] = messageBlockTwo[46] | messageBlockTwo[126];
    messageBlockTwo[23] = messageBlockTwo[40] & messageBlockTwo[75];
    messageBlockTwo[24] = messageBlockTwo[13] & messageBlockTwo[39];
    messageBlockTwo[25] = messageBlockTwo[68] ^ messageBlockTwo[117];
    messageBlockTwo[26] = messageBlockTwo[12] ^ messageBlockTwo[96];
    messageBlockTwo[27] = messageBlockTwo[121] & messageBlockTwo[22];
    messageBlockTwo[28] = messageBlockTwo[105] | messageBlockTwo[80];
    messageBlockTwo[29] = messageBlockTwo[59] ^ messageBlockTwo[90];
    messageBlockTwo[30] = messageBlockTwo[55] & messageBlockTwo[82];
    messageBlockTwo[31] = messageBlockTwo[39] & messageBlockTwo[48];
};

void secondBlockMutationThree() {
    messageBlockTwo[32] = messageBlockTwo[87] | messageBlockTwo[38];
    messageBlockTwo[33] = messageBlockTwo[90] | messageBlockTwo[113];
    messageBlockTwo[34] = messageBlockTwo[45] ^ messageBlockTwo[94];
    messageBlockTwo[35] = messageBlockTwo[24] & messageBlockTwo[76];
    messageBlockTwo[36] = messageBlockTwo[120] ^ messageBlockTwo[5];
    messageBlockTwo[37] = messageBlockTwo[42] & messageBlockTwo[123];
    messageBlockTwo[38] = messageBlockTwo[11] ^ messageBlockTwo[106];
    messageBlockTwo[39] = messageBlockTwo[62] | messageBlockTwo[11];
    messageBlockTwo[40] = messageBlockTwo[67] ^ messageBlockTwo[3];
    messageBlockTwo[41] = messageBlockTwo[15] & messageBlockTwo[56];
    messageBlockTwo[42] = messageBlockTwo[21] & messageBlockTwo[62];
    messageBlockTwo[43] = messageBlockTwo[104] ^ messageBlockTwo[28];
    messageBlockTwo[44] = messageBlockTwo[110] & messageBlockTwo[88];
    messageBlockTwo[45] = messageBlockTwo[70] | messageBlockTwo[18];
    messageBlockTwo[46] = messageBlockTwo[122] ^ messageBlockTwo[47];
    messageBlockTwo[47] = messageBlockTwo[76] | messageBlockTwo[71];
    messageBlockTwo[48] = messageBlockTwo[41] ^ messageBlockTwo[67];
};

void secondBlockMutationFour() {
    messageBlockTwo[49] = messageBlockTwo[86] & messageBlockTwo[21];
    messageBlockTwo[50] = messageBlockTwo[91] | messageBlockTwo[109];
    messageBlockTwo[51] = messageBlockTwo[102] | messageBlockTwo[65];
    messageBlockTwo[52] = messageBlockTwo[25] ^ messageBlockTwo[12];
    messageBlockTwo[53] = messageBlockTwo[10] & messageBlockTwo[124];
    messageBlockTwo[54] = messageBlockTwo[47] ^ messageBlockTwo[118];
    messageBlockTwo[55] = messageBlockTwo[63] & messageBlockTwo[15];
    messageBlockTwo[56] = messageBlockTwo[66] ^ messageBlockTwo[108];
    messageBlockTwo[57] = messageBlockTwo[112] ^ messageBlockTwo[114];
    messageBlockTwo[58] = messageBlockTwo[16] ^ messageBlockTwo[23];
    messageBlockTwo[59] = messageBlockTwo[111] & messageBlockTwo[97];
    messageBlockTwo[60] = messageBlockTwo[43] | messageBlockTwo[34];
    messageBlockTwo[61] = messageBlockTwo[103] | messageBlockTwo[41];
    messageBlockTwo[62] = messageBlockTwo[113] | messageBlockTwo[91];
    messageBlockTwo[63] = messageBlockTwo[20] & messageBlockTwo[84];
    messageBlockTwo[64] = messageBlockTwo[77] & messageBlockTwo[60];
};

void secondBlockMutationFive() {
    messageBlockTwo[65] = messageBlockTwo[2] ^ messageBlockTwo[74];
    messageBlockTwo[66] = messageBlockTwo[18] ^ messageBlockTwo[77];
    messageBlockTwo[67] = messageBlockTwo[35] & messageBlockTwo[79];
    messageBlockTwo[68] = messageBlockTwo[71] | messageBlockTwo[119];
    messageBlockTwo[69] = messageBlockTwo[9] ^ messageBlockTwo[37];
    messageBlockTwo[70] = messageBlockTwo[85] & messageBlockTwo[104];
    messageBlockTwo[71] = messageBlockTwo[31] & messageBlockTwo[1];
    messageBlockTwo[72] = messageBlockTwo[48] ^ messageBlockTwo[57];
    messageBlockTwo[73] = messageBlockTwo[73] | messageBlockTwo[87];
    messageBlockTwo[74] = messageBlockTwo[92] ^ messageBlockTwo[122];
    messageBlockTwo[75] = messageBlockTwo[17] & messageBlockTwo[9];
    messageBlockTwo[76] = messageBlockTwo[101] ^ messageBlockTwo[46];
    messageBlockTwo[77] = messageBlockTwo[115] ^ messageBlockTwo[72];
    messageBlockTwo[78] = messageBlockTwo[44] ^ messageBlockTwo[29];
    messageBlockTwo[79] = messageBlockTwo[78] | messageBlockTwo[110];
    messageBlockTwo[80] = messageBlockTwo[114] ^ messageBlockTwo[98];
    messageBlockTwo[81] = messageBlockTwo[26] & messageBlockTwo[20];
};
void secondBlockMutationSix() {
    messageBlockTwo[82] = messageBlockTwo[30] & messageBlockTwo[50];
    messageBlockTwo[83] = messageBlockTwo[64] & messageBlockTwo[53];
    messageBlockTwo[84] = messageBlockTwo[117] ^ messageBlockTwo[83];
    messageBlockTwo[85] = messageBlockTwo[7] & messageBlockTwo[70];
    messageBlockTwo[86] = messageBlockTwo[123] | messageBlockTwo[52];
    messageBlockTwo[87] = messageBlockTwo[93] | messageBlockTwo[16];
    messageBlockTwo[88] = messageBlockTwo[116] | messageBlockTwo[93];
    messageBlockTwo[89] = messageBlockTwo[79] | messageBlockTwo[105];
    messageBlockTwo[90] = messageBlockTwo[36] & messageBlockTwo[6];
    messageBlockTwo[91] = messageBlockTwo[19] ^ messageBlockTwo[120];
    messageBlockTwo[92] = messageBlockTwo[72] ^ messageBlockTwo[107];
    messageBlockTwo[93] = messageBlockTwo[3] & messageBlockTwo[24];
    messageBlockTwo[94] = messageBlockTwo[37] | messageBlockTwo[35];
    messageBlockTwo[95] = messageBlockTwo[27] & messageBlockTwo[85];
    messageBlockTwo[96] = messageBlockTwo[65] | messageBlockTwo[63];
    messageBlockTwo[97] = messageBlockTwo[34] & messageBlockTwo[10];
};

void secondBlockMutationSeven() {
    messageBlockTwo[98] = messageBlockTwo[29] & messageBlockTwo[99];
    messageBlockTwo[99] = messageBlockTwo[94] ^ messageBlockTwo[68];
    messageBlockTwo[100] = messageBlockTwo[49] & messageBlockTwo[42];
    messageBlockTwo[101] = messageBlockTwo[52] & messageBlockTwo[64];
    messageBlockTwo[102] = messageBlockTwo[54] | messageBlockTwo[112];
    messageBlockTwo[103] = messageBlockTwo[83] | messageBlockTwo[44];
    messageBlockTwo[104] = messageBlockTwo[100] & messageBlockTwo[121];
    messageBlockTwo[105] = messageBlockTwo[53] | messageBlockTwo[78];
    messageBlockTwo[106] = messageBlockTwo[125] & messageBlockTwo[58];
    messageBlockTwo[107] = messageBlockTwo[84] ^ messageBlockTwo[103];
    messageBlockTwo[108] = messageBlockTwo[28] & messageBlockTwo[102];
    messageBlockTwo[109] = messageBlockTwo[126] ^ messageBlockTwo[8];
    messageBlockTwo[110] = messageBlockTwo[4] & messageBlockTwo[111];
    messageBlockTwo[111] = messageBlockTwo[96] | messageBlockTwo[101];
    messageBlockTwo[112] = messageBlockTwo[124] ^ messageBlockTwo[73];
    messageBlockTwo[113] = messageBlockTwo[95] & messageBlockTwo[2];
    messageBlockTwo[114] = messageBlockTwo[80] ^ messageBlockTwo[30];
};

void secondBlockMutationEight() {
    messageBlockTwo[115] = messageBlockTwo[8] | messageBlockTwo[92];
    messageBlockTwo[116] = messageBlockTwo[50] | messageBlockTwo[19];
    messageBlockTwo[117] = messageBlockTwo[82] ^ messageBlockTwo[43];
    messageBlockTwo[118] = messageBlockTwo[98] & messageBlockTwo[45];
    messageBlockTwo[119] = messageBlockTwo[127] & messageBlockTwo[100];
    messageBlockTwo[120] = messageBlockTwo[32] ^ messageBlockTwo[86];
    messageBlockTwo[121] = messageBlockTwo[51] | messageBlockTwo[59];
    messageBlockTwo[122] = messageBlockTwo[99] | messageBlockTwo[17];
    messageBlockTwo[123] = messageBlockTwo[97] ^ messageBlockTwo[69];
    messageBlockTwo[124] = messageBlockTwo[5] | messageBlockTwo[51];
    messageBlockTwo[125] = messageBlockTwo[81] ^ messageBlockTwo[7];
    messageBlockTwo[126] = messageBlockTwo[33] | messageBlockTwo[36];
    messageBlockTwo[127] = messageBlockTwo[6] | messageBlockTwo[25];
};

//thirdBlock Mutation Series
void thirdBlockMutationOne() {
    messageBlockThree[0] = messageBlockThree[24] & messageBlockThree[23];
    messageBlockThree[1] = messageBlockThree[56] & messageBlockThree[38];
    messageBlockThree[2] = messageBlockThree[72] | messageBlockThree[55];
    messageBlockThree[3] = messageBlockThree[37] | messageBlockThree[111];
    messageBlockThree[4] = messageBlockThree[75] ^ messageBlockThree[116];
    messageBlockThree[5] = messageBlockThree[84] | messageBlockThree[125];
    messageBlockThree[6] = messageBlockThree[93] ^ messageBlockThree[110];
    messageBlockThree[7] = messageBlockThree[101] | messageBlockThree[9];
    messageBlockThree[8] = messageBlockThree[70] | messageBlockThree[59];
    messageBlockThree[9] = messageBlockThree[119] ^ messageBlockThree[86];
    messageBlockThree[10] = messageBlockThree[127] | messageBlockThree[66];
    messageBlockThree[11] = messageBlockThree[118] & messageBlockThree[85];
    messageBlockThree[12] = messageBlockThree[66] ^ messageBlockThree[118];
    messageBlockThree[13] = messageBlockThree[83] & messageBlockThree[34];
    messageBlockThree[14] = messageBlockThree[71] & messageBlockThree[56];
    messageBlockThree[15] = messageBlockThree[0] & messageBlockThree[60];
};

void thirdBlockMutationTwo() { 
    messageBlockThree[16] = messageBlockThree[23] ^ messageBlockThree[117];
    messageBlockThree[17] = messageBlockThree[103] | messageBlockThree[124];
    messageBlockThree[18] = messageBlockThree[51] | messageBlockThree[10];
    messageBlockThree[19] = messageBlockThree[5] ^ messageBlockThree[22];
    messageBlockThree[20] = messageBlockThree[25] | messageBlockThree[123];
    messageBlockThree[21] = messageBlockThree[57] & messageBlockThree[87];
    messageBlockThree[22] = messageBlockThree[35] ^ messageBlockThree[37];
    messageBlockThree[23] = messageBlockThree[3] | messageBlockThree[54];
    messageBlockThree[24] = messageBlockThree[111] ^ messageBlockThree[58];
    messageBlockThree[25] = messageBlockThree[102] & messageBlockThree[119];
    messageBlockThree[26] = messageBlockThree[49] ^ messageBlockThree[84];
    messageBlockThree[27] = messageBlockThree[82] ^ messageBlockThree[67];
    messageBlockThree[28] = messageBlockThree[92] & messageBlockThree[115];
    messageBlockThree[29] = messageBlockThree[74] ^ messageBlockThree[66];
    messageBlockThree[30] = messageBlockThree[120] ^ messageBlockThree[24];
    messageBlockThree[31] = messageBlockThree[1] & messageBlockThree[1];
};

void thirdBlockMutationThree() {
    messageBlockThree[32] = messageBlockThree[38] & messageBlockThree[39];
    messageBlockThree[33] = messageBlockThree[54] & messageBlockThree[35];
    messageBlockThree[34] = messageBlockThree[112] & messageBlockThree[76];
    messageBlockThree[35] = messageBlockThree[94] & messageBlockThree[14];
    messageBlockThree[36] = messageBlockThree[100] ^ messageBlockThree[32];
    messageBlockThree[37] = messageBlockThree[2] | messageBlockThree[99];
    messageBlockThree[38] = messageBlockThree[58] | messageBlockThree[12];
    messageBlockThree[39] = messageBlockThree[18] ^ messageBlockThree[109];
    messageBlockThree[40] = messageBlockThree[50] | messageBlockThree[88];
    messageBlockThree[41] = messageBlockThree[4] | messageBlockThree[21];
    messageBlockThree[42] = messageBlockThree[36] ^ messageBlockThree[45];
    messageBlockThree[43] = messageBlockThree[55] | messageBlockThree[83];
    messageBlockThree[44] = messageBlockThree[104] ^ messageBlockThree[108];
    messageBlockThree[45] = messageBlockThree[22] ^ messageBlockThree[52];
    messageBlockThree[46] = messageBlockThree[52] | messageBlockThree[57];
    messageBlockThree[47] = messageBlockThree[121] | messageBlockThree[100];
    messageBlockThree[48] = messageBlockThree[26] ^ messageBlockThree[112];
};

void thirdBlockMutationFour() {
    messageBlockThree[49] = messageBlockThree[34] & messageBlockThree[107];
    messageBlockThree[50] = messageBlockThree[67] ^ messageBlockThree[75];
    messageBlockThree[51] = messageBlockThree[39] & messageBlockThree[53];
    messageBlockThree[52] = messageBlockThree[27] ^ messageBlockThree[11];
    messageBlockThree[53] = messageBlockThree[73] ^ messageBlockThree[98];
    messageBlockThree[54] = messageBlockThree[81] ^ messageBlockThree[31];
    messageBlockThree[55] = messageBlockThree[91] & messageBlockThree[68];
    messageBlockThree[56] = messageBlockThree[19] | messageBlockThree[101];
    messageBlockThree[57] = messageBlockThree[85] | messageBlockThree[77];
    messageBlockThree[58] = messageBlockThree[47] | messageBlockThree[61];
    messageBlockThree[59] = messageBlockThree[105] & messageBlockThree[13];
    messageBlockThree[60] = messageBlockThree[59] ^ messageBlockThree[114];
    messageBlockThree[61] = messageBlockThree[95] & messageBlockThree[47];
    messageBlockThree[62] = messageBlockThree[53] ^ messageBlockThree[89];
    messageBlockThree[63] = messageBlockThree[80] | messageBlockThree[36];
    messageBlockThree[64] = messageBlockThree[6] ^ messageBlockThree[2];
};

void thirdBlockMutationFive() {
    messageBlockThree[65] = messageBlockThree[113] | messageBlockThree[19];
    messageBlockThree[66] = messageBlockThree[76] | messageBlockThree[20];
    messageBlockThree[67] = messageBlockThree[10] ^ messageBlockThree[30];
    messageBlockThree[68] = messageBlockThree[40] ^ messageBlockThree[74];
    messageBlockThree[69] = messageBlockThree[110] | messageBlockThree[106];
    messageBlockThree[70] = messageBlockThree[60] ^ messageBlockThree[94];
    messageBlockThree[71] = messageBlockThree[13] ^ messageBlockThree[82];
    messageBlockThree[72] = messageBlockThree[8] ^ messageBlockThree[62];
    messageBlockThree[73] = messageBlockThree[48] | messageBlockThree[48];
    messageBlockThree[74] = messageBlockThree[46] ^ messageBlockThree[78];
    messageBlockThree[75] = messageBlockThree[106] | messageBlockThree[97];
    messageBlockThree[76] = messageBlockThree[33] ^ messageBlockThree[90];
    messageBlockThree[77] = messageBlockThree[125] & messageBlockThree[40];
    messageBlockThree[78] = messageBlockThree[122] ^ messageBlockThree[29];
    messageBlockThree[79] = messageBlockThree[96] & messageBlockThree[46];
    messageBlockThree[80] = messageBlockThree[86] ^ messageBlockThree[69];
    messageBlockThree[81] = messageBlockThree[20] & messageBlockThree[105];
};

void thirdBlockMutationSix() {
    messageBlockThree[82] = messageBlockThree[7] ^ messageBlockThree[93];
    messageBlockThree[83] = messageBlockThree[28] & messageBlockThree[25];
    messageBlockThree[84] = messageBlockThree[41] | messageBlockThree[18];
    messageBlockThree[85] = messageBlockThree[61] | messageBlockThree[102];
    messageBlockThree[86] = messageBlockThree[115] ^ messageBlockThree[15];
    messageBlockThree[87] = messageBlockThree[123] & messageBlockThree[113];
    messageBlockThree[88] = messageBlockThree[78] ^ messageBlockThree[33];
    messageBlockThree[89] = messageBlockThree[45] | messageBlockThree[103];
    messageBlockThree[90] = messageBlockThree[107] | messageBlockThree[91];
    messageBlockThree[91] = messageBlockThree[124] ^ messageBlockThree[63];
    messageBlockThree[92] = messageBlockThree[21] | messageBlockThree[41];
    messageBlockThree[93] = messageBlockThree[114] ^ messageBlockThree[92];
    messageBlockThree[94] = messageBlockThree[9] | messageBlockThree[79];
    messageBlockThree[95] = messageBlockThree[97] | messageBlockThree[3];
    messageBlockThree[96] = messageBlockThree[87] & messageBlockThree[49];
    messageBlockThree[97] = messageBlockThree[77] & messageBlockThree[41];
};

void thirdBlockMutationSeven() {
    messageBlockThree[98] = messageBlockThree[42] ^ messageBlockThree[7];
    messageBlockThree[99] = messageBlockThree[108] ^ messageBlockThree[65];
    messageBlockThree[100] = messageBlockThree[12] | messageBlockThree[72];
    messageBlockThree[101] = messageBlockThree[32] ^ messageBlockThree[26];
    messageBlockThree[102] = messageBlockThree[62] | messageBlockThree[28];
    messageBlockThree[103] = messageBlockThree[126] ^ messageBlockThree[44];
    messageBlockThree[104] = messageBlockThree[44] & messageBlockThree[95];
    messageBlockThree[105] = messageBlockThree[29] & messageBlockThree[127];
    messageBlockThree[106] = messageBlockThree[89] ^ messageBlockThree[71];
    messageBlockThree[107] = messageBlockThree[116] | messageBlockThree[121];
    messageBlockThree[108] = messageBlockThree[30] | messageBlockThree[5];
    messageBlockThree[109] = messageBlockThree[98] | messageBlockThree[70];
    messageBlockThree[110] = messageBlockThree[79] ^ messageBlockThree[120];
    messageBlockThree[111] = messageBlockThree[68] & messageBlockThree[128];
    messageBlockThree[112] = messageBlockThree[43] ^ messageBlockThree[6];
    messageBlockThree[113] = messageBlockThree[16] & messageBlockThree[126];
    messageBlockThree[114] = messageBlockThree[11] ^ messageBlockThree[104];
};

void thirdBlockMutationEight() {
    messageBlockThree[115] = messageBlockThree[64] | messageBlockThree[80];
    messageBlockThree[116] = messageBlockThree[117] | messageBlockThree[96];
    messageBlockThree[117] = messageBlockThree[109] & messageBlockThree[43];
    messageBlockThree[118] = messageBlockThree[88] ^ messageBlockThree[16];
    messageBlockThree[119] = messageBlockThree[63] ^ messageBlockThree[50];
    messageBlockThree[120] = messageBlockThree[15] & messageBlockThree[64];
    messageBlockThree[121] = messageBlockThree[31] | messageBlockThree[81];
    messageBlockThree[122] = messageBlockThree[99] ^ messageBlockThree[73];
    messageBlockThree[123] = messageBlockThree[69] & messageBlockThree[51];
    messageBlockThree[124] = messageBlockThree[65] | messageBlockThree[27];
    messageBlockThree[125] = messageBlockThree[90] ^ messageBlockThree[4];
    messageBlockThree[126] = messageBlockThree[17] | messageBlockThree[17];
    messageBlockThree[127] = messageBlockThree[14] | messageBlockThree[8];
};

//fourthBlock Mutation Series
void forthBlockMutationOne() {
    messageBlockFour[0] = messageBlockFour[8] & messageBlockFour[47];
    messageBlockFour[1] = messageBlockFour[36] | messageBlockFour[55];
    messageBlockFour[2] = messageBlockFour[81] & messageBlockFour[24];
    messageBlockFour[3] = messageBlockFour[25] & messageBlockFour[65];
    messageBlockFour[4] = messageBlockFour[68] | messageBlockFour[9];
    messageBlockFour[5] = messageBlockFour[56] & messageBlockFour[45];
    messageBlockFour[6] = messageBlockFour[102] ^ messageBlockFour[33];
    messageBlockFour[7] = messageBlockFour[24] ^ messageBlockFour[15];
    messageBlockFour[8] = messageBlockFour[67] | messageBlockFour[85];
    messageBlockFour[9] = messageBlockFour[84] | messageBlockFour[95];
    messageBlockFour[10] = messageBlockFour[113] & messageBlockFour[108];
    messageBlockFour[11] = messageBlockFour[117] & messageBlockFour[118];
    messageBlockFour[12] = messageBlockFour[119] ^ messageBlockFour[128];
    messageBlockFour[13] = messageBlockFour[82] | messageBlockFour[54];
    messageBlockFour[14] = messageBlockFour[57] ^ messageBlockFour[74];
    messageBlockFour[15] = messageBlockFour[7] & messageBlockFour[116];
};

void forthBlockMutationTwo() {
    messageBlockFour[16] = messageBlockFour[9] ^ messageBlockFour[23];
    messageBlockFour[17] = messageBlockFour[114] | messageBlockFour[97];
    messageBlockFour[18] = messageBlockFour[66] ^ messageBlockFour[117];
    messageBlockFour[19] = messageBlockFour[80] & messageBlockFour[109];
    messageBlockFour[20] = messageBlockFour[101] ^ messageBlockFour[84];
    messageBlockFour[21] = messageBlockFour[116] | messageBlockFour[56];
    messageBlockFour[22] = messageBlockFour[23] | messageBlockFour[107];
    messageBlockFour[23] = messageBlockFour[118] ^ messageBlockFour[96];
    messageBlockFour[24] = messageBlockFour[69] & messageBlockFour[8];
    messageBlockFour[25] = messageBlockFour[108] & messageBlockFour[32];
    messageBlockFour[26] = messageBlockFour[37] ^ messageBlockFour[46];
    messageBlockFour[27] = messageBlockFour[65] ^ messageBlockFour[94];
    messageBlockFour[28] = messageBlockFour[6] | messageBlockFour[75];
    messageBlockFour[29] = messageBlockFour[55] | messageBlockFour[64];
    messageBlockFour[30] = messageBlockFour[103] ^ messageBlockFour[44];
    messageBlockFour[31] = messageBlockFour[120] & messageBlockFour[1];
};

void forthBlockMutationThree() {
    messageBlockFour[32] = messageBlockFour[26] | messageBlockFour[14];
    messageBlockFour[33] = messageBlockFour[79] ^ messageBlockFour[83];
    messageBlockFour[34] = messageBlockFour[83] ^ messageBlockFour[105];
    messageBlockFour[35] = messageBlockFour[39] | messageBlockFour[73];
    messageBlockFour[36] = messageBlockFour[97] ^ messageBlockFour[114];
    messageBlockFour[37] = messageBlockFour[10] ^ messageBlockFour[86];
    messageBlockFour[38] = messageBlockFour[12] & messageBlockFour[34];
    messageBlockFour[39] = messageBlockFour[64] & messageBlockFour[122];
    messageBlockFour[40] = messageBlockFour[70] ^ messageBlockFour[124];
    messageBlockFour[41] = messageBlockFour[22] | messageBlockFour[115];
    messageBlockFour[42] = messageBlockFour[58] | messageBlockFour[76];
    messageBlockFour[43] = messageBlockFour[99] ^ messageBlockFour[7];
    messageBlockFour[44] = messageBlockFour[5] & messageBlockFour[110];
    messageBlockFour[45] = messageBlockFour[54] ^ messageBlockFour[22];
    messageBlockFour[46] = messageBlockFour[104] & messageBlockFour[106];
    messageBlockFour[47] = messageBlockFour[98] ^ messageBlockFour[43];
    messageBlockFour[48] = messageBlockFour[78] | messageBlockFour[63];
};

void forthBlockMutationFour() {
    messageBlockFour[49] = messageBlockFour[47] ^ messageBlockFour[82];
    messageBlockFour[50] = messageBlockFour[109] | messageBlockFour[72];
    messageBlockFour[51] = messageBlockFour[115] ^ messageBlockFour[31];
    messageBlockFour[52] = messageBlockFour[11] & messageBlockFour[25];
    messageBlockFour[53] = messageBlockFour[38] & messageBlockFour[48];
    messageBlockFour[54] = messageBlockFour[112] ^ messageBlockFour[93];
    messageBlockFour[55] = messageBlockFour[21] ^ messageBlockFour[10];
    messageBlockFour[56] = messageBlockFour[86] | messageBlockFour[104];
    messageBlockFour[57] = messageBlockFour[111] | messageBlockFour[52];
    messageBlockFour[58] = messageBlockFour[105] ^ messageBlockFour[98];
    messageBlockFour[59] = messageBlockFour[4] & messageBlockFour[113];
    messageBlockFour[60] = messageBlockFour[110] & messageBlockFour[111];
    messageBlockFour[61] = messageBlockFour[48] ^ messageBlockFour[87];
    messageBlockFour[62] = messageBlockFour[106] | messageBlockFour[53];
    messageBlockFour[63] = messageBlockFour[85] & messageBlockFour[71];
    messageBlockFour[64] = messageBlockFour[107] ^ messageBlockFour[16];
};

void forthBlockMutationFive() {
    messageBlockFour[65] = messageBlockFour[13] & messageBlockFour[3];
    messageBlockFour[66] = messageBlockFour[40] | messageBlockFour[62];
    messageBlockFour[67] = messageBlockFour[63] & messageBlockFour[26];
    messageBlockFour[68] = messageBlockFour[46] ^ messageBlockFour[92];
    messageBlockFour[69] = messageBlockFour[19] | messageBlockFour[21];
    messageBlockFour[70] = messageBlockFour[71] | messageBlockFour[51];
    messageBlockFour[71] = messageBlockFour[59] ^ messageBlockFour[39];
    messageBlockFour[72] = messageBlockFour[35] & messageBlockFour[42];
    messageBlockFour[73] = messageBlockFour[87] ^ messageBlockFour[6];
    messageBlockFour[74] = messageBlockFour[29] & messageBlockFour[77];
    messageBlockFour[75] = messageBlockFour[53] ^ messageBlockFour[81];
    messageBlockFour[76] = messageBlockFour[72] ^ messageBlockFour[88];
    messageBlockFour[77] = messageBlockFour[96] | messageBlockFour[13];
    messageBlockFour[78] = messageBlockFour[27] | messageBlockFour[70];
    messageBlockFour[79] = messageBlockFour[100] & messageBlockFour[100];
    messageBlockFour[80] = messageBlockFour[121] ^ messageBlockFour[112];
    messageBlockFour[81] = messageBlockFour[49] & messageBlockFour[66];
};

void forthBlockMutationSix() {
    messageBlockFour[82] = messageBlockFour[14] ^ messageBlockFour[30];
    messageBlockFour[83] = messageBlockFour[20] ^ messageBlockFour[61];
    messageBlockFour[84] = messageBlockFour[3] ^ messageBlockFour[103];
    messageBlockFour[85] = messageBlockFour[77] & messageBlockFour[2];
    messageBlockFour[86] = messageBlockFour[18] ^ messageBlockFour[49];
    messageBlockFour[87] = messageBlockFour[31] ^ messageBlockFour[27];
    messageBlockFour[88] = messageBlockFour[34] | messageBlockFour[91];
    messageBlockFour[89] = messageBlockFour[41] ^ messageBlockFour[11];
    messageBlockFour[90] = messageBlockFour[73] & messageBlockFour[99];
    messageBlockFour[91] = messageBlockFour[89] & messageBlockFour[57];
    messageBlockFour[92] = messageBlockFour[28] ^ messageBlockFour[38];
    messageBlockFour[93] = messageBlockFour[90] | messageBlockFour[35];
    messageBlockFour[94] = messageBlockFour[88] ^ messageBlockFour[20];
    messageBlockFour[95] = messageBlockFour[50] & messageBlockFour[80];
    messageBlockFour[96] = messageBlockFour[17] ^ messageBlockFour[78];
    messageBlockFour[97] = messageBlockFour[15] | messageBlockFour[40];
};

void forthBlockMutationSeven() {
    messageBlockFour[98] = messageBlockFour[43] ^ messageBlockFour[89];
    messageBlockFour[99] = messageBlockFour[95] | messageBlockFour[101];
    messageBlockFour[100] = messageBlockFour[44] ^ messageBlockFour[60];
    messageBlockFour[101] = messageBlockFour[123] | messageBlockFour[67];
    messageBlockFour[102] = messageBlockFour[125] & messageBlockFour[126];
    messageBlockFour[103] = messageBlockFour[42] | messageBlockFour[58];
    messageBlockFour[104] = messageBlockFour[52] ^ messageBlockFour[5];
    messageBlockFour[105] = messageBlockFour[74] ^ messageBlockFour[18];
    messageBlockFour[106] = messageBlockFour[60] | messageBlockFour[69];
    messageBlockFour[107] = messageBlockFour[30] | messageBlockFour[120];
    messageBlockFour[108] = messageBlockFour[124] ^ messageBlockFour[123];
    messageBlockFour[109] = messageBlockFour[122] & messageBlockFour[28];
    messageBlockFour[110] = messageBlockFour[16] & messageBlockFour[127];
    messageBlockFour[111] = messageBlockFour[62] ^ messageBlockFour[125];
    messageBlockFour[112] = messageBlockFour[91] & messageBlockFour[17];
    messageBlockFour[113] = messageBlockFour[51] | messageBlockFour[121];
    messageBlockFour[114] = messageBlockFour[1] ^ messageBlockFour[36];
};

void forthBlockMutationEight() {
    messageBlockFour[115] = messageBlockFour[93] ^ messageBlockFour[119];
    messageBlockFour[116] = messageBlockFour[32] ^ messageBlockFour[102];
    messageBlockFour[117] = messageBlockFour[127] ^ messageBlockFour[19];
    messageBlockFour[118] = messageBlockFour[126] & messageBlockFour[50];
    messageBlockFour[119] = messageBlockFour[45] ^ messageBlockFour[90];
    messageBlockFour[120] = messageBlockFour[94] | messageBlockFour[68];
    messageBlockFour[121] = messageBlockFour[0] ^ messageBlockFour[12];
    messageBlockFour[122] = messageBlockFour[75] ^ messageBlockFour[79];
    messageBlockFour[123] = messageBlockFour[92] ^ messageBlockFour[59];
    messageBlockFour[124] = messageBlockFour[61] | messageBlockFour[41];
    messageBlockFour[125] = messageBlockFour[2] ^ messageBlockFour[29];
    messageBlockFour[126] = messageBlockFour[76] & messageBlockFour[37];
    messageBlockFour[127] = messageBlockFour[33] & messageBlockFour[4];
};

//Alter series of 00000000's
//Not working as intended
void mutateZeroSet() {
for (auto i: messageBlockFour) {
    bitset<8> setZero = 00000000;
    bitset<8> setOne = 0111111111;
    if (i == setZero) {
        messageBlockFour.push_back(setOne);
    };
};
};

void printMessageBlocks() {
    cout << endl << "First messageBlock:" << endl;
for (auto i: messageBlockOne) {
        cout << i << ' ';
    };
cout << endl << "Second messageBlock:" << endl;
for (auto i: messageBlockTwo) {
        cout << i << ' ';
    };
cout << endl << "Third messageBlock:" << endl;
for (auto i: messageBlockThree) {
        cout << i << ' ';
    };
cout << endl << "Fourth messageBlock:" << endl;
for (auto i: messageBlockFour) {
        cout << i << ' ';
    };
};


void concatenateMessageBlocks() {
concatenateFirstBlock.reserve(messageBlockOne.size() + messageBlockTwo.size());
concatenateFirstBlock.insert(concatenateFirstBlock.end(), messageBlockOne.begin(), messageBlockOne.end());
concatenateFirstBlock.insert(concatenateFirstBlock.end(), messageBlockTwo.begin(), messageBlockTwo.end());

concatenateSecondBlock.reserve(messageBlockThree.size() + messageBlockFour.size());
concatenateSecondBlock.insert(concatenateSecondBlock.end(), messageBlockThree.begin(), messageBlockThree.end());
concatenateSecondBlock.insert(concatenateSecondBlock.end(), messageBlockFour.begin(), messageBlockFour.end());

//Concate second block first to further change message index
finalMessageDigest.reserve(concatenateSecondBlock.size() + concatenateFirstBlock.size());
finalMessageDigest.insert(finalMessageDigest.end(), concatenateSecondBlock.begin(), concatenateSecondBlock.end());
finalMessageDigest.insert(finalMessageDigest.end(), concatenateFirstBlock.begin(), concatenateFirstBlock.end());
};

//Convert any leading ones to zeroes
void alterLeadingZero() {
for (auto i: finalMessageDigest) {
    i |= 0 << 7;
};
};

void vectorToString() {
    //Convert finalMessageDigest vector to string
    for(size_t i=0; i < finalMessageDigest.size(); ++i){
        if (i != 0)
        messageStream << finalMessageDigest[i];
    }
    string stringMessage = messageStream.str();
    //Convert binary to char
    stringstream sstream(stringMessage);
    while(sstream.good()){
        bitset<8> messageBytes;
        sstream >> messageBytes;
        char c = char(messageBytes.to_ulong());
        completeMessageDigest += c;
    };
    //Clear white space, remove none ASCII Characters and resize the hash
    completeMessageDigest.erase(remove_if(completeMessageDigest.begin(), completeMessageDigest.end(), ::isspace), completeMessageDigest.end());
    completeMessageDigest.erase(remove_if(completeMessageDigest.begin(), completeMessageDigest.end(), [](char c) {return !isalpha(c), !iswalnum(c);}), completeMessageDigest.end());
    completeMessageDigest.resize(256);
    cout << "messageDigest:" << endl << completeMessageDigest << endl;
};
};

int main() {
    message newMessage;
    mainVector newVector;
    newMessage.gatherMessage();
    newMessage.convertToASCII();
    cout << "Binary Message: \n";
    newVector.binaryConverter(newMessage.messageInput);
    cout << "\nASCII to Binary Message: \n";
    newVector.binaryConverter(newMessage.messageASCIIString);
    cout << "\npadValue: \n";
    newVector.binaryConverter(newMessage.padValueString);
    newVector.padmessageDigest();
    newVector.reversemessageDigest();
    newVector.messageDigestSize();
    newVector.messageDigestTrim();
    newVector.firstMutation();
    newVector.messageDigestTrim();
    cout << "First Mutation Wave" << endl;
    newVector.printmessageDigest();
    newVector.divideMessageDigest();
    cout << endl << "Second Mutation Wave";
    newVector.firstBlockMutationOne();
    newVector.firstBlockMutationTwo();
    newVector.firstBlockMutationThree();
    newVector.firstBlockMutationFour();
    newVector.firstBlockMutationFive();
    newVector.firstBlockMutationSix();
    newVector.firstBlockMutationSeven();
    newVector.firstBlockMutationEight();
    newVector.secondBlockMutationOne();
    newVector.secondBlockMutationTwo();
    newVector.secondBlockMutationThree();
    newVector.secondBlockMutationFour();
    newVector.secondBlockMutationFive();
    newVector.secondBlockMutationSix();
    newVector.secondBlockMutationSeven();
    newVector.secondBlockMutationEight();
    newVector.thirdBlockMutationOne();
    newVector.thirdBlockMutationTwo();
    newVector.thirdBlockMutationThree();
    newVector.thirdBlockMutationFour();
    newVector.thirdBlockMutationFive();
    newVector.thirdBlockMutationSix();
    newVector.thirdBlockMutationSeven();
    newVector.thirdBlockMutationEight();
    newVector.forthBlockMutationOne();
    newVector.forthBlockMutationTwo();
    newVector.forthBlockMutationThree();
    newVector.forthBlockMutationFour();
    newVector.forthBlockMutationFive();
    newVector.forthBlockMutationSix();
    newVector.forthBlockMutationSeven();
    newVector.forthBlockMutationEight();
    newVector.mutateZeroSet();
    newVector.printMessageBlocks();
    newVector.concatenateMessageBlocks();
    newVector.alterLeadingZero();
    cout << endl;
    newVector.messageDigestSize();
    newVector.vectorToString();
    //Pause until keystroke
    system("pause");
    return 0;
};
