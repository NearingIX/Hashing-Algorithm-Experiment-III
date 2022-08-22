//Hashing Algorithm Experiment III

#include <iostream>
#include <bitset>
#include <vector>
#include <algorithm>
#include <string>
#include <cmath>
#include <iterator>
#include <sstream>
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

void secondMutation(){
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
void firstBlockMutation() {
    messageBlockOne[0] = messageBlockOne[0] ^ messageBlockOne[8];
    messageBlockOne[1] = messageBlockOne[1] | messageBlockOne[72];
    messageBlockOne[2] = messageBlockOne[2] & messageBlockOne[41];
    messageBlockOne[3] = messageBlockOne[3] ^ messageBlockOne[58];
    messageBlockOne[4] = messageBlockOne[4] & messageBlockOne[99];
    messageBlockOne[5] = messageBlockOne[5] & messageBlockOne[47];
    messageBlockOne[6] = messageBlockOne[6] & messageBlockOne[28];
    messageBlockOne[7] = messageBlockOne[7] ^ messageBlockOne[76];
    messageBlockOne[8] = messageBlockOne[8] | messageBlockOne[82];
    messageBlockOne[9] = messageBlockOne[9] ^ messageBlockOne[91];
    messageBlockOne[10] = messageBlockOne[10] ^ messageBlockOne[16];
    messageBlockOne[11] = messageBlockOne[11] & messageBlockOne[71];
    messageBlockOne[12] = messageBlockOne[12] | messageBlockOne[81];
    messageBlockOne[13] = messageBlockOne[13] & messageBlockOne[108];
    messageBlockOne[14] = messageBlockOne[14] ^ messageBlockOne[52];
    messageBlockOne[15] = messageBlockOne[15] | messageBlockOne[26];
    messageBlockOne[16] = messageBlockOne[16] ^ messageBlockOne[35];
    messageBlockOne[17] = messageBlockOne[17] & messageBlockOne[74];
    messageBlockOne[18] = messageBlockOne[18] ^ messageBlockOne[98];
    messageBlockOne[19] = messageBlockOne[19] | messageBlockOne[121];
    messageBlockOne[20] = messageBlockOne[20] ^ messageBlockOne[117];
    messageBlockOne[21] = messageBlockOne[21] | messageBlockOne[73];
    messageBlockOne[22] = messageBlockOne[22] & messageBlockOne[37];
    messageBlockOne[23] = messageBlockOne[23] ^ messageBlockOne[123];
    messageBlockOne[24] = messageBlockOne[24] ^ messageBlockOne[42];
    messageBlockOne[25] = messageBlockOne[25] & messageBlockOne[6];
    messageBlockOne[26] = messageBlockOne[26] ^ messageBlockOne[30];
    messageBlockOne[27] = messageBlockOne[27] ^ messageBlockOne[119];
    messageBlockOne[28] = messageBlockOne[28] | messageBlockOne[122];
    messageBlockOne[29] = messageBlockOne[29] | messageBlockOne[118];
    messageBlockOne[30] = messageBlockOne[30] & messageBlockOne[92];
    messageBlockOne[31] = messageBlockOne[31] & messageBlockOne[57];
};

void firstBlockMutationTwo() {
    messageBlockOne[32] = messageBlockOne[32] | messageBlockOne[46];
    messageBlockOne[33] = messageBlockOne[33] | messageBlockOne[34];
    messageBlockOne[34] = messageBlockOne[34] & messageBlockOne[7];
    messageBlockOne[35] = messageBlockOne[35] | messageBlockOne[15];
    messageBlockOne[36] = messageBlockOne[36] | messageBlockOne[80];
    messageBlockOne[37] = messageBlockOne[37] ^ messageBlockOne[90];
    messageBlockOne[38] = messageBlockOne[38] & messageBlockOne[126];
    messageBlockOne[39] = messageBlockOne[39] | messageBlockOne[124];
    messageBlockOne[40] = messageBlockOne[40] ^ messageBlockOne[56];
    messageBlockOne[41] = messageBlockOne[41] ^ messageBlockOne[61];
    messageBlockOne[42] = messageBlockOne[42] ^ messageBlockOne[59];
    messageBlockOne[43] = messageBlockOne[43] | messageBlockOne[27];
    messageBlockOne[44] = messageBlockOne[44] & messageBlockOne[83];
    messageBlockOne[45] = messageBlockOne[45] & messageBlockOne[107];
    messageBlockOne[46] = messageBlockOne[46] ^ messageBlockOne[48];
    messageBlockOne[47] = messageBlockOne[47] & messageBlockOne[100];
    messageBlockOne[48] = messageBlockOne[48] ^ messageBlockOne[53];
    messageBlockOne[49] = messageBlockOne[49] | messageBlockOne[84];
    messageBlockOne[50] = messageBlockOne[50] | messageBlockOne[17];
    messageBlockOne[51] = messageBlockOne[51] ^ messageBlockOne[5];
    messageBlockOne[52] = messageBlockOne[52] ^ messageBlockOne[36];
    messageBlockOne[53] = messageBlockOne[53] | messageBlockOne[19];
    messageBlockOne[54] = messageBlockOne[54] | messageBlockOne[10];
    messageBlockOne[55] = messageBlockOne[55] | messageBlockOne[29];
    messageBlockOne[56] = messageBlockOne[56] & messageBlockOne[38];
    messageBlockOne[57] = messageBlockOne[57] | messageBlockOne[75];
    messageBlockOne[58] = messageBlockOne[58] ^ messageBlockOne[79];
    messageBlockOne[59] = messageBlockOne[59] ^ messageBlockOne[9];
    messageBlockOne[60] = messageBlockOne[60] & messageBlockOne[68];
    messageBlockOne[61] = messageBlockOne[61] & messageBlockOne[62];
    messageBlockOne[62] = messageBlockOne[62] & messageBlockOne[33];
    messageBlockOne[63] = messageBlockOne[63] ^ messageBlockOne[18];
    messageBlockOne[64] = messageBlockOne[64] & messageBlockOne[4];
};

void firstBlockMutationThree() {
    messageBlockOne[65] = messageBlockOne[65] | messageBlockOne[70];
    messageBlockOne[66] = messageBlockOne[66] & messageBlockOne[69];
    messageBlockOne[67] = messageBlockOne[67] & messageBlockOne[93];
    messageBlockOne[68] = messageBlockOne[68] | messageBlockOne[97];
    messageBlockOne[69] = messageBlockOne[69] ^ messageBlockOne[112];
    messageBlockOne[70] = messageBlockOne[70] & messageBlockOne[89];
    messageBlockOne[71] = messageBlockOne[71] ^ messageBlockOne[24];
    messageBlockOne[72] = messageBlockOne[72] & messageBlockOne[14];
    messageBlockOne[73] = messageBlockOne[73] | messageBlockOne[111];
    messageBlockOne[74] = messageBlockOne[74] & messageBlockOne[125];
    messageBlockOne[75] = messageBlockOne[75] ^ messageBlockOne[67];
    messageBlockOne[76] = messageBlockOne[76] | messageBlockOne[51];
    messageBlockOne[77] = messageBlockOne[77] ^ messageBlockOne[78];
    messageBlockOne[78] = messageBlockOne[78] & messageBlockOne[120];
    messageBlockOne[79] = messageBlockOne[79] ^ messageBlockOne[113];
    messageBlockOne[80] = messageBlockOne[80] ^ messageBlockOne[23];
    messageBlockOne[81] = messageBlockOne[81] | messageBlockOne[25];
    messageBlockOne[82] = messageBlockOne[82] | messageBlockOne[63];
    messageBlockOne[83] = messageBlockOne[83] & messageBlockOne[49];
    messageBlockOne[84] = messageBlockOne[84] ^ messageBlockOne[85];
    messageBlockOne[85] = messageBlockOne[85] | messageBlockOne[106];
    messageBlockOne[86] = messageBlockOne[86] | messageBlockOne[54];
    messageBlockOne[87] = messageBlockOne[87] ^ messageBlockOne[20];
    messageBlockOne[88] = messageBlockOne[88] & messageBlockOne[11];
    messageBlockOne[89] = messageBlockOne[89] & messageBlockOne[77];
    messageBlockOne[90] = messageBlockOne[90] ^ messageBlockOne[109];
    messageBlockOne[91] = messageBlockOne[91] | messageBlockOne[94];
    messageBlockOne[92] = messageBlockOne[92] | messageBlockOne[2];
    messageBlockOne[93] = messageBlockOne[93] ^ messageBlockOne[43];
    messageBlockOne[94] = messageBlockOne[94] & messageBlockOne[101];
    messageBlockOne[95] = messageBlockOne[95] ^ messageBlockOne[86];
    messageBlockOne[96] = messageBlockOne[96] ^ messageBlockOne[66];
    messageBlockOne[97] = messageBlockOne[97] | messageBlockOne[32]; 
};

void firstBlockMutationFour() {
    messageBlockOne[98] = messageBlockOne[98] ^ messageBlockOne[105];
    messageBlockOne[99] = messageBlockOne[99] | messageBlockOne[115];
    messageBlockOne[100] = messageBlockOne[100] | messageBlockOne[128];
    messageBlockOne[101] = messageBlockOne[101] ^ messageBlockOne[114];
    messageBlockOne[102] = messageBlockOne[102] & messageBlockOne[87];
    messageBlockOne[103] = messageBlockOne[103] & messageBlockOne[13];
    messageBlockOne[104] = messageBlockOne[104] ^ messageBlockOne[95];
    messageBlockOne[105] = messageBlockOne[105] | messageBlockOne[60];
    messageBlockOne[106] = messageBlockOne[106] ^ messageBlockOne[3];
    messageBlockOne[107] = messageBlockOne[107] | messageBlockOne[102];
    messageBlockOne[108] = messageBlockOne[108] ^ messageBlockOne[45];
    messageBlockOne[109] = messageBlockOne[109] & messageBlockOne[39];
    messageBlockOne[110] = messageBlockOne[110] ^ messageBlockOne[44];
    messageBlockOne[111] = messageBlockOne[111] & messageBlockOne[65];
    messageBlockOne[112] = messageBlockOne[112] ^ messageBlockOne[104];
    messageBlockOne[113] = messageBlockOne[113] & messageBlockOne[127];
    messageBlockOne[114] = messageBlockOne[114] | messageBlockOne[116];
    messageBlockOne[115] = messageBlockOne[115] | messageBlockOne[110];
    messageBlockOne[116] = messageBlockOne[116] ^ messageBlockOne[55];
    messageBlockOne[117] = messageBlockOne[117] ^ messageBlockOne[50];
    messageBlockOne[118] = messageBlockOne[118] ^ messageBlockOne[21];
    messageBlockOne[119] = messageBlockOne[119] & messageBlockOne[103];
    messageBlockOne[120] = messageBlockOne[120] & messageBlockOne[88];
    messageBlockOne[121] = messageBlockOne[121] ^ messageBlockOne[1];
    messageBlockOne[122] = messageBlockOne[122] & messageBlockOne[22];
    messageBlockOne[123] = messageBlockOne[123] ^ messageBlockOne[96];
    messageBlockOne[124] = messageBlockOne[124] ^ messageBlockOne[64];
    messageBlockOne[125] = messageBlockOne[125] | messageBlockOne[40];
    messageBlockOne[126] = messageBlockOne[126] | messageBlockOne[31];
    messageBlockOne[127] = messageBlockOne[127] | messageBlockOne[12];
};

//secondBlock Mutation Series
void secondBlockMutation() {
    messageBlockTwo[0] = messageBlockTwo[0] | messageBlockTwo[14];
    messageBlockTwo[1] = messageBlockTwo[1] | messageBlockTwo[27];
    messageBlockTwo[2] = messageBlockTwo[2] ^ messageBlockTwo[33];
    messageBlockTwo[3] = messageBlockTwo[3] & messageBlockTwo[40];
    messageBlockTwo[4] = messageBlockTwo[4] ^ messageBlockTwo[55];
    messageBlockTwo[5] = messageBlockTwo[5] & messageBlockTwo[116];
    messageBlockTwo[6] = messageBlockTwo[6] & messageBlockTwo[128];
    messageBlockTwo[7] = messageBlockTwo[7] & messageBlockTwo[49];
    messageBlockTwo[8] = messageBlockTwo[8] & messageBlockTwo[26];
    messageBlockTwo[9] = messageBlockTwo[9] | messageBlockTwo[125];
    messageBlockTwo[10] = messageBlockTwo[10] | messageBlockTwo[32];
    messageBlockTwo[11] = messageBlockTwo[11] & messageBlockTwo[89];
    messageBlockTwo[12] = messageBlockTwo[12] & messageBlockTwo[95];
    messageBlockTwo[13] = messageBlockTwo[13] ^ messageBlockTwo[54];
    messageBlockTwo[14] = messageBlockTwo[14] | messageBlockTwo[4];
    messageBlockTwo[15] = messageBlockTwo[15] & messageBlockTwo[31];
    messageBlockTwo[16] = messageBlockTwo[16] | messageBlockTwo[127];
    messageBlockTwo[17] = messageBlockTwo[17] & messageBlockTwo[115];
    messageBlockTwo[18] = messageBlockTwo[18] | messageBlockTwo[61];
    messageBlockTwo[19] = messageBlockTwo[19] ^ messageBlockTwo[13];
    messageBlockTwo[20] = messageBlockTwo[20] | messageBlockTwo[81];
    messageBlockTwo[21] = messageBlockTwo[21] ^ messageBlockTwo[66];
    messageBlockTwo[22] = messageBlockTwo[22] | messageBlockTwo[126];
    messageBlockTwo[23] = messageBlockTwo[23] & messageBlockTwo[75];
    messageBlockTwo[24] = messageBlockTwo[24] & messageBlockTwo[39];
    messageBlockTwo[25] = messageBlockTwo[25] ^ messageBlockTwo[117];
    messageBlockTwo[26] = messageBlockTwo[26] ^ messageBlockTwo[96];
    messageBlockTwo[27] = messageBlockTwo[27] & messageBlockTwo[22];
    messageBlockTwo[28] = messageBlockTwo[28] | messageBlockTwo[80];
    messageBlockTwo[29] = messageBlockTwo[29] ^ messageBlockTwo[90];
    messageBlockTwo[30] = messageBlockTwo[30] & messageBlockTwo[82];
    messageBlockTwo[31] = messageBlockTwo[31] & messageBlockTwo[48];
};

void secondBlockMutationTwo() {
    messageBlockTwo[32] = messageBlockTwo[32] | messageBlockTwo[38];
    messageBlockTwo[33] = messageBlockTwo[33] | messageBlockTwo[113];
    messageBlockTwo[34] = messageBlockTwo[34] ^ messageBlockTwo[94];
    messageBlockTwo[35] = messageBlockTwo[35] & messageBlockTwo[76];
    messageBlockTwo[36] = messageBlockTwo[36] ^ messageBlockTwo[5];
    messageBlockTwo[37] = messageBlockTwo[37] & messageBlockTwo[123];
    messageBlockTwo[38] = messageBlockTwo[38] ^ messageBlockTwo[106];
    messageBlockTwo[39] = messageBlockTwo[39] | messageBlockTwo[11];
    messageBlockTwo[40] = messageBlockTwo[40] ^ messageBlockTwo[3];
    messageBlockTwo[41] = messageBlockTwo[41] & messageBlockTwo[56];
    messageBlockTwo[42] = messageBlockTwo[42] & messageBlockTwo[62];
    messageBlockTwo[43] = messageBlockTwo[43] ^ messageBlockTwo[28];
    messageBlockTwo[44] = messageBlockTwo[44] & messageBlockTwo[88];
    messageBlockTwo[45] = messageBlockTwo[45] | messageBlockTwo[18];
    messageBlockTwo[46] = messageBlockTwo[46] ^ messageBlockTwo[47];
    messageBlockTwo[47] = messageBlockTwo[47] | messageBlockTwo[71];
    messageBlockTwo[48] = messageBlockTwo[48] ^ messageBlockTwo[67];
    messageBlockTwo[49] = messageBlockTwo[49] & messageBlockTwo[21];
    messageBlockTwo[50] = messageBlockTwo[50] | messageBlockTwo[109];
    messageBlockTwo[51] = messageBlockTwo[51] | messageBlockTwo[65];
    messageBlockTwo[52] = messageBlockTwo[52] ^ messageBlockTwo[12];
    messageBlockTwo[53] = messageBlockTwo[53] & messageBlockTwo[124];
    messageBlockTwo[54] = messageBlockTwo[54] ^ messageBlockTwo[118];
    messageBlockTwo[55] = messageBlockTwo[55] & messageBlockTwo[15];
    messageBlockTwo[56] = messageBlockTwo[56] ^ messageBlockTwo[108];
    messageBlockTwo[57] = messageBlockTwo[57] ^ messageBlockTwo[114];
    messageBlockTwo[58] = messageBlockTwo[58] ^ messageBlockTwo[23];
    messageBlockTwo[59] = messageBlockTwo[59] & messageBlockTwo[97];
    messageBlockTwo[60] = messageBlockTwo[60] | messageBlockTwo[34];
    messageBlockTwo[61] = messageBlockTwo[61] | messageBlockTwo[41];
    messageBlockTwo[62] = messageBlockTwo[62] | messageBlockTwo[91];
    messageBlockTwo[63] = messageBlockTwo[63] & messageBlockTwo[84];
    messageBlockTwo[64] = messageBlockTwo[64] & messageBlockTwo[60];
};

void secondBlockMutationThree() {
    messageBlockTwo[65] = messageBlockTwo[65] ^ messageBlockTwo[74];
    messageBlockTwo[66] = messageBlockTwo[66] ^ messageBlockTwo[77];
    messageBlockTwo[67] = messageBlockTwo[67] & messageBlockTwo[79];
    messageBlockTwo[68] = messageBlockTwo[68] | messageBlockTwo[119];
    messageBlockTwo[69] = messageBlockTwo[69] ^ messageBlockTwo[37];
    messageBlockTwo[70] = messageBlockTwo[70] & messageBlockTwo[104];
    messageBlockTwo[71] = messageBlockTwo[71] & messageBlockTwo[1];
    messageBlockTwo[72] = messageBlockTwo[72] ^ messageBlockTwo[57];
    messageBlockTwo[73] = messageBlockTwo[73] | messageBlockTwo[87];
    messageBlockTwo[74] = messageBlockTwo[74] ^ messageBlockTwo[122];
    messageBlockTwo[75] = messageBlockTwo[75] & messageBlockTwo[9];
    messageBlockTwo[76] = messageBlockTwo[76] ^ messageBlockTwo[46];
    messageBlockTwo[77] = messageBlockTwo[77] ^ messageBlockTwo[72];
    messageBlockTwo[78] = messageBlockTwo[78] ^ messageBlockTwo[29];
    messageBlockTwo[79] = messageBlockTwo[79] | messageBlockTwo[110];
    messageBlockTwo[80] = messageBlockTwo[80] ^ messageBlockTwo[98];
    messageBlockTwo[81] = messageBlockTwo[81] & messageBlockTwo[20];
    messageBlockTwo[82] = messageBlockTwo[82] & messageBlockTwo[50];
    messageBlockTwo[83] = messageBlockTwo[83] & messageBlockTwo[53];
    messageBlockTwo[84] = messageBlockTwo[84] ^ messageBlockTwo[83];
    messageBlockTwo[85] = messageBlockTwo[85] & messageBlockTwo[70];
    messageBlockTwo[86] = messageBlockTwo[86] | messageBlockTwo[52];
    messageBlockTwo[87] = messageBlockTwo[87] | messageBlockTwo[16];
    messageBlockTwo[88] = messageBlockTwo[88] | messageBlockTwo[93];
    messageBlockTwo[89] = messageBlockTwo[89] | messageBlockTwo[105];
    messageBlockTwo[90] = messageBlockTwo[90] & messageBlockTwo[6];
    messageBlockTwo[91] = messageBlockTwo[91] ^ messageBlockTwo[120];
    messageBlockTwo[92] = messageBlockTwo[92] ^ messageBlockTwo[107];
    messageBlockTwo[93] = messageBlockTwo[93] & messageBlockTwo[24];
    messageBlockTwo[94] = messageBlockTwo[94] | messageBlockTwo[35];
    messageBlockTwo[95] = messageBlockTwo[95] & messageBlockTwo[85];
    messageBlockTwo[96] = messageBlockTwo[96] | messageBlockTwo[63];
    messageBlockTwo[97] = messageBlockTwo[97] & messageBlockTwo[10];
};

void secondBlockMutationFour() {
    messageBlockTwo[98] = messageBlockTwo[98] & messageBlockTwo[99];
    messageBlockTwo[99] = messageBlockTwo[99] ^ messageBlockTwo[68];
    messageBlockTwo[100] = messageBlockTwo[100] & messageBlockTwo[42];
    messageBlockTwo[101] = messageBlockTwo[101] & messageBlockTwo[64];
    messageBlockTwo[102] = messageBlockTwo[102] | messageBlockTwo[112];
    messageBlockTwo[103] = messageBlockTwo[103] | messageBlockTwo[44];
    messageBlockTwo[104] = messageBlockTwo[104] & messageBlockTwo[121];
    messageBlockTwo[105] = messageBlockTwo[105] | messageBlockTwo[78];
    messageBlockTwo[106] = messageBlockTwo[106] & messageBlockTwo[58];
    messageBlockTwo[107] = messageBlockTwo[107] ^ messageBlockTwo[103];
    messageBlockTwo[108] = messageBlockTwo[108] & messageBlockTwo[102];
    messageBlockTwo[109] = messageBlockTwo[109] ^ messageBlockTwo[8];
    messageBlockTwo[110] = messageBlockTwo[110] & messageBlockTwo[111];
    messageBlockTwo[111] = messageBlockTwo[111] | messageBlockTwo[101];
    messageBlockTwo[112] = messageBlockTwo[112] ^ messageBlockTwo[73];
    messageBlockTwo[113] = messageBlockTwo[113] & messageBlockTwo[2];
    messageBlockTwo[114] = messageBlockTwo[114] ^ messageBlockTwo[30];
    messageBlockTwo[115] = messageBlockTwo[115] | messageBlockTwo[92];
    messageBlockTwo[116] = messageBlockTwo[116] | messageBlockTwo[19];
    messageBlockTwo[117] = messageBlockTwo[117] ^ messageBlockTwo[43];
    messageBlockTwo[118] = messageBlockTwo[118] & messageBlockTwo[45];
    messageBlockTwo[119] = messageBlockTwo[119] & messageBlockTwo[100];
    messageBlockTwo[120] = messageBlockTwo[120] ^ messageBlockTwo[86];
    messageBlockTwo[121] = messageBlockTwo[121] | messageBlockTwo[59];
    messageBlockTwo[122] = messageBlockTwo[122] | messageBlockTwo[17];
    messageBlockTwo[123] = messageBlockTwo[123] ^ messageBlockTwo[69];
    messageBlockTwo[124] = messageBlockTwo[124] | messageBlockTwo[51];
    messageBlockTwo[125] = messageBlockTwo[125] ^ messageBlockTwo[7];
    messageBlockTwo[126] = messageBlockTwo[126] | messageBlockTwo[36];
    messageBlockTwo[127] = messageBlockTwo[127] | messageBlockTwo[25];
};

//thirdBlock Mutation Series

void thirdBlockMutation() {
    messageBlockThree[0] = messageBlockThree[5] & messageBlockThree[23];
    messageBlockThree[1] = messageBlockThree[1] & messageBlockThree[38];
    messageBlockThree[2] = messageBlockThree[2] | messageBlockThree[55];
    messageBlockThree[3] = messageBlockThree[3] | messageBlockThree[111];
    messageBlockThree[4] = messageBlockThree[4] ^ messageBlockThree[116];
    messageBlockThree[5] = messageBlockThree[0] | messageBlockThree[125];
    messageBlockThree[6] = messageBlockThree[6] ^ messageBlockThree[110];
    messageBlockThree[7] = messageBlockThree[7] | messageBlockThree[9];
    messageBlockThree[8] = messageBlockThree[8] | messageBlockThree[59];
    messageBlockThree[9] = messageBlockThree[9] ^ messageBlockThree[86];
    messageBlockThree[10] = messageBlockThree[10] | messageBlockThree[66];
    messageBlockThree[11] = messageBlockThree[11] & messageBlockThree[85];
    messageBlockThree[12] = messageBlockThree[12] ^ messageBlockThree[118];
    messageBlockThree[13] = messageBlockThree[13] & messageBlockThree[34];
    messageBlockThree[14] = messageBlockThree[14] & messageBlockThree[56];
    messageBlockThree[15] = messageBlockThree[15] & messageBlockThree[60];
    messageBlockThree[16] = messageBlockThree[16] ^ messageBlockThree[117];
    messageBlockThree[17] = messageBlockThree[17] | messageBlockThree[124];
    messageBlockThree[18] = messageBlockThree[18] | messageBlockThree[10];
    messageBlockThree[19] = messageBlockThree[19] ^ messageBlockThree[22];
    messageBlockThree[20] = messageBlockThree[20] | messageBlockThree[123];
    messageBlockThree[21] = messageBlockThree[21] & messageBlockThree[87];
    messageBlockThree[22] = messageBlockThree[22] ^ messageBlockThree[37];
    messageBlockThree[23] = messageBlockThree[23] | messageBlockThree[54];
    messageBlockThree[24] = messageBlockThree[24] ^ messageBlockThree[58];
    messageBlockThree[25] = messageBlockThree[25] & messageBlockThree[119];
    messageBlockThree[26] = messageBlockThree[26] ^ messageBlockThree[84];
    messageBlockThree[27] = messageBlockThree[27] ^ messageBlockThree[67];
    messageBlockThree[28] = messageBlockThree[28] & messageBlockThree[115];
    messageBlockThree[29] = messageBlockThree[29] ^ messageBlockThree[66];
    messageBlockThree[30] = messageBlockThree[30] ^ messageBlockThree[24];
    messageBlockThree[31] = messageBlockThree[31] & messageBlockThree[1];
};

void thirdBlockMutationTwo() {
    messageBlockThree[32] = messageBlockThree[32] & messageBlockThree[39];
    messageBlockThree[33] = messageBlockThree[33] & messageBlockThree[35];
    messageBlockThree[34] = messageBlockThree[34] & messageBlockThree[76];
    messageBlockThree[35] = messageBlockThree[35] & messageBlockThree[14];
    messageBlockThree[36] = messageBlockThree[36] ^ messageBlockThree[32];
    messageBlockThree[37] = messageBlockThree[37] | messageBlockThree[99];
    messageBlockThree[38] = messageBlockThree[38] | messageBlockThree[12];
    messageBlockThree[39] = messageBlockThree[39] ^ messageBlockThree[109];
    messageBlockThree[40] = messageBlockThree[40] | messageBlockThree[88];
    messageBlockThree[41] = messageBlockThree[41] | messageBlockThree[21];
    messageBlockThree[42] = messageBlockThree[42] ^ messageBlockThree[45];
    messageBlockThree[43] = messageBlockThree[43] | messageBlockThree[83];
    messageBlockThree[44] = messageBlockThree[44] ^ messageBlockThree[108];
    messageBlockThree[45] = messageBlockThree[45] ^ messageBlockThree[52];
    messageBlockThree[46] = messageBlockThree[46] | messageBlockThree[57];
    messageBlockThree[47] = messageBlockThree[47] | messageBlockThree[100];
    messageBlockThree[48] = messageBlockThree[48] ^ messageBlockThree[112];
    messageBlockThree[49] = messageBlockThree[49] & messageBlockThree[107];
    messageBlockThree[50] = messageBlockThree[50] ^ messageBlockThree[75];
    messageBlockThree[51] = messageBlockThree[51] & messageBlockThree[53];
    messageBlockThree[52] = messageBlockThree[52] ^ messageBlockThree[11];
    messageBlockThree[53] = messageBlockThree[53] ^ messageBlockThree[98];
    messageBlockThree[54] = messageBlockThree[54] ^ messageBlockThree[31];
    messageBlockThree[55] = messageBlockThree[55] & messageBlockThree[68];
    messageBlockThree[56] = messageBlockThree[56] | messageBlockThree[101];
    messageBlockThree[57] = messageBlockThree[57] | messageBlockThree[77];
    messageBlockThree[58] = messageBlockThree[58] | messageBlockThree[61];
    messageBlockThree[59] = messageBlockThree[59] & messageBlockThree[13];
    messageBlockThree[60] = messageBlockThree[60] ^ messageBlockThree[114];
    messageBlockThree[61] = messageBlockThree[61] & messageBlockThree[47];
    messageBlockThree[62] = messageBlockThree[62] ^ messageBlockThree[89];
    messageBlockThree[63] = messageBlockThree[63] | messageBlockThree[36];
    messageBlockThree[64] = messageBlockThree[64] ^ messageBlockThree[2];
};

void thirdBlockMutationThree() {
    messageBlockThree[65] = messageBlockThree[65] | messageBlockThree[19];
    messageBlockThree[66] = messageBlockThree[66] | messageBlockThree[20];
    messageBlockThree[67] = messageBlockThree[67] ^ messageBlockThree[30];
    messageBlockThree[68] = messageBlockThree[68] ^ messageBlockThree[74];
    messageBlockThree[69] = messageBlockThree[69] | messageBlockThree[106];
    messageBlockThree[70] = messageBlockThree[70] ^ messageBlockThree[94];
    messageBlockThree[71] = messageBlockThree[71] ^ messageBlockThree[82];
    messageBlockThree[72] = messageBlockThree[72] ^ messageBlockThree[62];
    messageBlockThree[73] = messageBlockThree[73] | messageBlockThree[48];
    messageBlockThree[74] = messageBlockThree[74] ^ messageBlockThree[78];
    messageBlockThree[75] = messageBlockThree[75] | messageBlockThree[97];
    messageBlockThree[76] = messageBlockThree[76] ^ messageBlockThree[90];
    messageBlockThree[77] = messageBlockThree[77] & messageBlockThree[40];
    messageBlockThree[78] = messageBlockThree[78] ^ messageBlockThree[29];
    messageBlockThree[79] = messageBlockThree[79] & messageBlockThree[46];
    messageBlockThree[80] = messageBlockThree[80] ^ messageBlockThree[69];
    messageBlockThree[81] = messageBlockThree[81] & messageBlockThree[105];
    messageBlockThree[82] = messageBlockThree[82] ^ messageBlockThree[93];
    messageBlockThree[83] = messageBlockThree[83] & messageBlockThree[25];
    messageBlockThree[84] = messageBlockThree[84] | messageBlockThree[18];
    messageBlockThree[85] = messageBlockThree[85] | messageBlockThree[102];
    messageBlockThree[86] = messageBlockThree[86] ^ messageBlockThree[15];
    messageBlockThree[87] = messageBlockThree[87] & messageBlockThree[113];
    messageBlockThree[88] = messageBlockThree[88] ^ messageBlockThree[33];
    messageBlockThree[89] = messageBlockThree[89] | messageBlockThree[103];
    messageBlockThree[90] = messageBlockThree[90] | messageBlockThree[91];
    messageBlockThree[91] = messageBlockThree[91] ^ messageBlockThree[63];
    messageBlockThree[92] = messageBlockThree[92] | messageBlockThree[41];
    messageBlockThree[93] = messageBlockThree[93] ^ messageBlockThree[92];
    messageBlockThree[94] = messageBlockThree[94] | messageBlockThree[79];
    messageBlockThree[95] = messageBlockThree[95] | messageBlockThree[3];
    messageBlockThree[96] = messageBlockThree[96] & messageBlockThree[49];
    messageBlockThree[97] = messageBlockThree[97] & messageBlockThree[41]; 
};

void thirdBlockMutationFour() {
    messageBlockThree[98] = messageBlockThree[98] ^ messageBlockThree[7];
    messageBlockThree[99] = messageBlockThree[99] ^ messageBlockThree[65];
    messageBlockThree[100] = messageBlockThree[100] | messageBlockThree[72];
    messageBlockThree[101] = messageBlockThree[101] ^ messageBlockThree[26];
    messageBlockThree[102] = messageBlockThree[102] | messageBlockThree[28];
    messageBlockThree[103] = messageBlockThree[103] ^ messageBlockThree[44];
    messageBlockThree[104] = messageBlockThree[104] & messageBlockThree[95];
    messageBlockThree[105] = messageBlockThree[105] & messageBlockThree[127];
    messageBlockThree[106] = messageBlockThree[106] ^ messageBlockThree[71];
    messageBlockThree[107] = messageBlockThree[107] | messageBlockThree[121];
    messageBlockThree[108] = messageBlockThree[108] | messageBlockThree[5];
    messageBlockThree[109] = messageBlockThree[109] | messageBlockThree[70];
    messageBlockThree[110] = messageBlockThree[110] ^ messageBlockThree[120];
    messageBlockThree[111] = messageBlockThree[111] & messageBlockThree[128];
    messageBlockThree[112] = messageBlockThree[112] ^ messageBlockThree[6];
    messageBlockThree[113] = messageBlockThree[113] & messageBlockThree[126];
    messageBlockThree[114] = messageBlockThree[114] ^ messageBlockThree[104];
    messageBlockThree[115] = messageBlockThree[115] | messageBlockThree[80];
    messageBlockThree[116] = messageBlockThree[116] | messageBlockThree[96];
    messageBlockThree[117] = messageBlockThree[117] & messageBlockThree[43];
    messageBlockThree[118] = messageBlockThree[118] ^ messageBlockThree[16];
    messageBlockThree[119] = messageBlockThree[119] ^ messageBlockThree[50];
    messageBlockThree[120] = messageBlockThree[120] & messageBlockThree[64];
    messageBlockThree[121] = messageBlockThree[121] | messageBlockThree[81];
    messageBlockThree[122] = messageBlockThree[122] ^ messageBlockThree[73];
    messageBlockThree[123] = messageBlockThree[123] & messageBlockThree[51];
    messageBlockThree[124] = messageBlockThree[124] | messageBlockThree[27];
    messageBlockThree[125] = messageBlockThree[125] ^ messageBlockThree[4];
    messageBlockThree[126] = messageBlockThree[126] | messageBlockThree[17];
    messageBlockThree[127] = messageBlockThree[127] | messageBlockThree[8];
};

//fourthBlock Mutation Series
void forthBlockMutation() {
    messageBlockFour[0] = messageBlockFour[0] & messageBlockFour[47];
    messageBlockFour[1] = messageBlockFour[1] | messageBlockFour[55];
    messageBlockFour[2] = messageBlockFour[2] & messageBlockFour[24];
    messageBlockFour[3] = messageBlockFour[3] & messageBlockFour[65];
    messageBlockFour[4] = messageBlockFour[4] | messageBlockFour[9];
    messageBlockFour[5] = messageBlockFour[5] & messageBlockFour[45];
    messageBlockFour[6] = messageBlockFour[6] ^ messageBlockFour[33];
    messageBlockFour[7] = messageBlockFour[7] ^ messageBlockFour[15];
    messageBlockFour[8] = messageBlockFour[8] | messageBlockFour[85];
    messageBlockFour[9] = messageBlockFour[9] | messageBlockFour[95];
    messageBlockFour[10] = messageBlockFour[10] & messageBlockFour[108];
    messageBlockFour[11] = messageBlockFour[11] & messageBlockFour[118];
    messageBlockFour[12] = messageBlockFour[12] ^ messageBlockFour[128];
    messageBlockFour[13] = messageBlockFour[13] | messageBlockFour[54];
    messageBlockFour[14] = messageBlockFour[14] ^ messageBlockFour[74];
    messageBlockFour[15] = messageBlockFour[15] & messageBlockFour[116];
    messageBlockFour[16] = messageBlockFour[16] ^ messageBlockFour[23];
    messageBlockFour[17] = messageBlockFour[17] | messageBlockFour[97];
    messageBlockFour[18] = messageBlockFour[18] ^ messageBlockFour[117];
    messageBlockFour[19] = messageBlockFour[19] & messageBlockFour[109];
    messageBlockFour[20] = messageBlockFour[20] ^ messageBlockFour[84];
    messageBlockFour[21] = messageBlockFour[21] | messageBlockFour[56];
    messageBlockFour[22] = messageBlockFour[22] | messageBlockFour[107];
    messageBlockFour[23] = messageBlockFour[23] ^ messageBlockFour[96];
    messageBlockFour[24] = messageBlockFour[24] & messageBlockFour[8];
    messageBlockFour[25] = messageBlockFour[25] & messageBlockFour[32];
    messageBlockFour[26] = messageBlockFour[26] ^ messageBlockFour[46];
    messageBlockFour[27] = messageBlockFour[27] ^ messageBlockFour[94];
    messageBlockFour[28] = messageBlockFour[28] | messageBlockFour[75];
    messageBlockFour[29] = messageBlockFour[29] | messageBlockFour[64];
    messageBlockFour[30] = messageBlockFour[30] ^ messageBlockFour[44];
    messageBlockFour[31] = messageBlockFour[31] & messageBlockFour[1];
};

void forthBlockMutationTwo() {
    messageBlockFour[32] = messageBlockFour[32] | messageBlockFour[14];
    messageBlockFour[33] = messageBlockFour[33] ^ messageBlockFour[83];
    messageBlockFour[34] = messageBlockFour[34] ^ messageBlockFour[105];
    messageBlockFour[35] = messageBlockFour[35] | messageBlockFour[73];
    messageBlockFour[36] = messageBlockFour[36] ^ messageBlockFour[114];
    messageBlockFour[37] = messageBlockFour[37] ^ messageBlockFour[86];
    messageBlockFour[38] = messageBlockFour[38] & messageBlockFour[34];
    messageBlockFour[39] = messageBlockFour[39] & messageBlockFour[122];
    messageBlockFour[40] = messageBlockFour[40] ^ messageBlockFour[124];
    messageBlockFour[41] = messageBlockFour[41] | messageBlockFour[115];
    messageBlockFour[42] = messageBlockFour[42] | messageBlockFour[76];
    messageBlockFour[43] = messageBlockFour[43] ^ messageBlockFour[7];
    messageBlockFour[44] = messageBlockFour[44] & messageBlockFour[110];
    messageBlockFour[45] = messageBlockFour[45] ^ messageBlockFour[22];
    messageBlockFour[46] = messageBlockFour[46] & messageBlockFour[106];
    messageBlockFour[47] = messageBlockFour[47] ^ messageBlockFour[43];
    messageBlockFour[48] = messageBlockFour[48] | messageBlockFour[63];
    messageBlockFour[49] = messageBlockFour[49] ^ messageBlockFour[82];
    messageBlockFour[50] = messageBlockFour[50] | messageBlockFour[72];
    messageBlockFour[51] = messageBlockFour[51] ^ messageBlockFour[31];
    messageBlockFour[52] = messageBlockFour[52] & messageBlockFour[25];
    messageBlockFour[53] = messageBlockFour[53] & messageBlockFour[48];
    messageBlockFour[54] = messageBlockFour[54] ^ messageBlockFour[93];
    messageBlockFour[55] = messageBlockFour[55] ^ messageBlockFour[10];
    messageBlockFour[56] = messageBlockFour[56] | messageBlockFour[104];
    messageBlockFour[57] = messageBlockFour[57] | messageBlockFour[52];
    messageBlockFour[58] = messageBlockFour[58] ^ messageBlockFour[98];
    messageBlockFour[59] = messageBlockFour[59] & messageBlockFour[113];
    messageBlockFour[60] = messageBlockFour[60] & messageBlockFour[111];
    messageBlockFour[61] = messageBlockFour[61] ^ messageBlockFour[87];
    messageBlockFour[62] = messageBlockFour[62] | messageBlockFour[53];
    messageBlockFour[63] = messageBlockFour[63] & messageBlockFour[71];
    messageBlockFour[64] = messageBlockFour[64] ^ messageBlockFour[16];
};

void forthBlockMutationThree() {
    messageBlockFour[65] = messageBlockFour[65] & messageBlockFour[3];
    messageBlockFour[66] = messageBlockFour[66] | messageBlockFour[62];
    messageBlockFour[67] = messageBlockFour[67] & messageBlockFour[26];
    messageBlockFour[68] = messageBlockFour[68] ^ messageBlockFour[92];
    messageBlockFour[69] = messageBlockFour[69] | messageBlockFour[21];
    messageBlockFour[70] = messageBlockFour[70] | messageBlockFour[51];
    messageBlockFour[71] = messageBlockFour[71] ^ messageBlockFour[39];
    messageBlockFour[72] = messageBlockFour[72] & messageBlockFour[42];
    messageBlockFour[73] = messageBlockFour[73] ^ messageBlockFour[6];
    messageBlockFour[74] = messageBlockFour[74] & messageBlockFour[77];
    messageBlockFour[75] = messageBlockFour[75] ^ messageBlockFour[81];
    messageBlockFour[76] = messageBlockFour[76] ^ messageBlockFour[88];
    messageBlockFour[77] = messageBlockFour[77] | messageBlockFour[13];
    messageBlockFour[78] = messageBlockFour[78] | messageBlockFour[70];
    messageBlockFour[79] = messageBlockFour[79] & messageBlockFour[100];
    messageBlockFour[80] = messageBlockFour[80] ^ messageBlockFour[112];
    messageBlockFour[81] = messageBlockFour[81] & messageBlockFour[66];
    messageBlockFour[82] = messageBlockFour[82] ^ messageBlockFour[30];
    messageBlockFour[83] = messageBlockFour[83] ^ messageBlockFour[61];
    messageBlockFour[84] = messageBlockFour[84] ^ messageBlockFour[103];
    messageBlockFour[85] = messageBlockFour[85] & messageBlockFour[2];
    messageBlockFour[86] = messageBlockFour[86] ^ messageBlockFour[49];
    messageBlockFour[87] = messageBlockFour[87] ^ messageBlockFour[27];
    messageBlockFour[88] = messageBlockFour[88] | messageBlockFour[91];
    messageBlockFour[89] = messageBlockFour[89] ^ messageBlockFour[11];
    messageBlockFour[90] = messageBlockFour[90] & messageBlockFour[99];
    messageBlockFour[91] = messageBlockFour[91] & messageBlockFour[57];
    messageBlockFour[92] = messageBlockFour[92] ^ messageBlockFour[38];
    messageBlockFour[93] = messageBlockFour[93] | messageBlockFour[35];
    messageBlockFour[94] = messageBlockFour[94] ^ messageBlockFour[20];
    messageBlockFour[95] = messageBlockFour[95] & messageBlockFour[80];
    messageBlockFour[96] = messageBlockFour[96] ^ messageBlockFour[78];
    messageBlockFour[97] = messageBlockFour[97] | messageBlockFour[40];
};

void forthBlockMutationFour() {
    messageBlockFour[98] = messageBlockFour[98] ^ messageBlockFour[89];
    messageBlockFour[99] = messageBlockFour[99] | messageBlockFour[101];
    messageBlockFour[100] = messageBlockFour[100] ^ messageBlockFour[60];
    messageBlockFour[101] = messageBlockFour[101] | messageBlockFour[67];
    messageBlockFour[102] = messageBlockFour[102] & messageBlockFour[126];
    messageBlockFour[103] = messageBlockFour[103] | messageBlockFour[58];
    messageBlockFour[104] = messageBlockFour[104] ^ messageBlockFour[5];
    messageBlockFour[105] = messageBlockFour[105] ^ messageBlockFour[18];
    messageBlockFour[106] = messageBlockFour[106] | messageBlockFour[69];
    messageBlockFour[107] = messageBlockFour[107] | messageBlockFour[120];
    messageBlockFour[108] = messageBlockFour[108] ^ messageBlockFour[123];
    messageBlockFour[109] = messageBlockFour[109] & messageBlockFour[28];
    messageBlockFour[110] = messageBlockFour[110] & messageBlockFour[127];
    messageBlockFour[111] = messageBlockFour[111] ^ messageBlockFour[125];
    messageBlockFour[112] = messageBlockFour[112] & messageBlockFour[17];
    messageBlockFour[113] = messageBlockFour[113] | messageBlockFour[121];
    messageBlockFour[114] = messageBlockFour[114] ^ messageBlockFour[36];
    messageBlockFour[115] = messageBlockFour[115] ^ messageBlockFour[119];
    messageBlockFour[116] = messageBlockFour[116] ^ messageBlockFour[102];
    messageBlockFour[117] = messageBlockFour[117] ^ messageBlockFour[19];
    messageBlockFour[118] = messageBlockFour[118] & messageBlockFour[50];
    messageBlockFour[119] = messageBlockFour[119] ^ messageBlockFour[90];
    messageBlockFour[120] = messageBlockFour[120] | messageBlockFour[68];
    messageBlockFour[121] = messageBlockFour[121] ^ messageBlockFour[12];
    messageBlockFour[122] = messageBlockFour[122] ^ messageBlockFour[79];
    messageBlockFour[123] = messageBlockFour[123] ^ messageBlockFour[59];
    messageBlockFour[124] = messageBlockFour[124] | messageBlockFour[41];
    messageBlockFour[125] = messageBlockFour[125] ^ messageBlockFour[29];
    messageBlockFour[126] = messageBlockFour[126] & messageBlockFour[37];
    messageBlockFour[127] = messageBlockFour[127] & messageBlockFour[4];
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
firstBlock.reserve(messageBlockOne.size() + messageBlockTwo.size());
firstBlock.insert(firstBlock.end(), messageBlockOne.begin(), messageBlockOne.end());
firstBlock.insert(firstBlock.end(), messageBlockTwo.begin(), messageBlockTwo.end());

secondBlock.reserve(messageBlockThree.size() + messageBlockFour.size());
secondBlock.insert(secondBlock.end(), messageBlockThree.begin(), messageBlockThree.end());
secondBlock.insert(secondBlock.end(), messageBlockFour.begin(), messageBlockFour.end());

finalMessageDigest.reserve(firstBlock.size() + secondBlock.size());
finalMessageDigest.insert(finalMessageDigest.end(), firstBlock.begin(), firstBlock.end());
finalMessageDigest.insert(finalMessageDigest.end(), secondBlock.begin(), secondBlock.end());
};

//Convert any leading ones to zeroes
void alterLeadingZero() {
for (auto i: messageDigest) {
    i |= 0 << 7;
};
};

//Convert finalMessageDigest vector to string
//Maybe break into two functions
void vectorToString() {
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
    //Remove leading uniform characters
    completeMessageDigest.erase(0,7);
    //Clear white space
    completeMessageDigest.erase(remove_if(completeMessageDigest.begin(), completeMessageDigest.end(), ::isspace), completeMessageDigest.end());
    cout << "binaryStringMessageDigest:" << endl << completeMessageDigest << endl;
    
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
    newVector.secondMutation();
    newVector.messageDigestTrim();
    cout << endl << "Second Mutation: " << endl;
    newVector.printmessageDigest();
    newVector.divideMessageDigest();
    newVector.firstBlockMutation();
    newVector.firstBlockMutationTwo();
    newVector.firstBlockMutationThree();
    newVector.firstBlockMutationFour();
    newVector.secondBlockMutation();
    newVector.secondBlockMutationTwo();
    newVector.secondBlockMutationThree();
    newVector.secondBlockMutationFour();
    newVector.thirdBlockMutation();
    newVector.thirdBlockMutationTwo();
    newVector.thirdBlockMutationThree();
    newVector.thirdBlockMutationFour();
    newVector.forthBlockMutation();
    newVector.forthBlockMutationTwo();
    newVector.forthBlockMutationThree();
    newVector.forthBlockMutationFour();
    newVector.printMessageBlocks();
    newVector.concatenateMessageBlocks();
    newVector.alterLeadingZero();
    cout << endl << "Final Message " << endl;
    newVector.printmessageDigest();
    cout << endl;
    newVector.messageDigestSize();
    newVector.vectorToString();
    return 0;
};
