/**
* This file is part of CCM-SLAM.
*
* Copyright (C): Patrik Schmuck <pschmuck at ethz dot ch> (ETH Zurich)
* For more information see <https://github.com/patriksc/CCM-SLAM>
*
* CCM-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* CCM-SLAM is based in the monocular version of ORB-SLAM2 by Raúl Mur-Artal.
* CCM-SLAM partially re-uses modules of ORB-SLAM2 in modified or unmodified condition.
* For more information see <https://github.com/raulmur/ORB_SLAM2>.
*
* CCM-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with CCM-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <cslam/Database.h>

namespace cslam {

KeyFrameDatabase::KeyFrameDatabase(const vocptr pVoc):
    mpVoc(pVoc)
{
    mvInvertedFile.resize((*pVoc).size());

    cout << "+++++ KeyFrame Database Initialized +++++" << endl;
}

void KeyFrameDatabase::add(kfptr pKF)
{
    unique_lock<mutex> lock(mMutex);

    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}

void KeyFrameDatabase::erase(kfptr pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        list<kfptr> &lKFs =   mvInvertedFile[vit->first];

        for(list<kfptr>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}

vector<KeyFrameDatabase::kfptr> KeyFrameDatabase::DetectLoopCandidates(kfptr pKF, float minScore)
{
    //// 得到与该关键帧连接（>15个共视地图点）的关键帧(没有排序的)
    set<kfptr> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<kfptr> lKFsSharingWords;
    //得到所有的在gai地图中的关键帧
    std::map<idpair,kfptr> mpAllKfsInMap = pKF->GetMapptr()->GetMmpKeyFrames();

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            list<kfptr> &lKFs =   mvInvertedFile[vit->first];

            for(list<kfptr>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                kfptr pKFi=*lit;

                if(pKFi->mId == pKF->mId) continue;

                std::map<idpair,kfptr>::iterator mit = mpAllKfsInMap.find(pKFi->mId);
                if(mit == mpAllKfsInMap.end()) continue; //Only consider KFs that belong to the same map

                if(!(pKFi->mLoopQuery==pKF->mId))
                {
                    pKFi->mnLoopWords=0;
                    if(!spConnectedKeyFrames.count(pKFi))
                    {
                        pKFi->mLoopQuery=pKF->mId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector<kfptr>();

    list<pair<float,kfptr> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<kfptr>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for(list<kfptr>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        kfptr pKFi = *lit;

        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;

            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<kfptr>();

    list<pair<float,kfptr> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    for(list<pair<float,kfptr> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        kfptr pKFi = it->second;
        vector<kfptr> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        kfptr pBestKF = pKFi;
        for(vector<kfptr>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            kfptr pKF2 = *vit;

            if(pKF2->mLoopQuery==pKF->mId && pKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pKF2->mLoopScore;
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    set<kfptr> spAlreadyAddedKF;
    vector<kfptr> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for(list<pair<float,kfptr> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            kfptr pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;
}

vector<KeyFrameDatabase::kfptr> KeyFrameDatabase::DetectMapMatchCandidates(kfptr pKF, float minScore, mapptr pMap)
{
    list<kfptr> lKFsSharingWords;
    //在所有关键帧中找到所有拥有共同单词，却和当前帧来自不同客户端的帧，并标记其匹配帧为当前帧，放进---lKFsSharingWords
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            list<kfptr> &lKFs =   mvInvertedFile[vit->first];

            for(list<kfptr>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                kfptr pKFi=*lit;

                if(!(pKFi->mMatchQuery==pKF->mId))
                {
                    pKFi->mnLoopWords=0;

                    if(!pMap->msuAssClients.count(pKFi->mId.second))
                    {
                        pKFi->mMatchQuery=pKF->mId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector<kfptr>();

    list<pair<float,kfptr> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    //更新最大单词的数量
    int maxCommonWords=0;
    for(list<kfptr>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }
    //最小的是最大的0.8倍
    int minCommonWords = maxCommonWords*0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    //计算单词数大于最小值的帧计算其与当前帧的分数，将得分和关键帧组成对应数据结构放入 --lScoreAndMatch
    for(list<kfptr>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        kfptr pKFi = *lit;

        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;

            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<kfptr>();

    list<pair<float,kfptr> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    for(list<pair<float,kfptr> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        kfptr pKFi = it->second;
        //找到lScoreAndMatch每个帧的前十五个共视帧
        vector<kfptr> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        kfptr pBestKF = pKFi;
        //如果前十五个共视帧的某个帧与当前帧匹配了而且其单词数量大于最小值，贡献分数给 accScore，并得到最高分数及对应关键帧
        for(vector<kfptr>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            kfptr pKF2 = *vit;

            if(pKF2->mMatchQuery==pKF->mId && pKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pKF2->mLoopScore;
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }
        // lAccScoreAndMatch存放一级共视某一帧的总分数和最高分
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    //设定0.75倍的最高总分为最低分
    float minScoreToRetain = 0.75f*bestAccScore;

    set<kfptr> spAlreadyAddedKF;
    vector<kfptr> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for(list<pair<float,kfptr> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        //如果得分>最低总分将对应关键帧加入vpLoopCandidates
        if(it->first>minScoreToRetain)
        {
            kfptr pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;
}

vector<KeyFrameDatabase::kfptr> KeyFrameDatabase::DetectRelocalizationCandidates(Frame &F)
{
    list<kfptr> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=F.mBowVec.begin(), vend=F.mBowVec.end(); vit != vend; vit++)
        {
            list<kfptr> &lKFs =   mvInvertedFile[vit->first];

            for(list<kfptr>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                kfptr pKFi=*lit;
                if(pKFi->mRelocQuery!=F.mId)
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mRelocQuery=F.mId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return vector<kfptr>();

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<kfptr>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    list<pair<float,kfptr> > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    for(list<kfptr>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        kfptr pKFi = *lit;

        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(F.mBowVec,pKFi->mBowVec);
            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<kfptr>();

    list<pair<float,kfptr> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(list<pair<float,kfptr> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        kfptr pKFi = it->second;
        vector<kfptr> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        kfptr pBestKF = pKFi;
        for(vector<kfptr>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            kfptr pKF2 = *vit;
            if(pKF2->mRelocQuery!=F.mId)
                continue;

            accScore+=pKF2->mRelocScore;
            if(pKF2->mRelocScore>bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;
    set<kfptr> spAlreadyAddedKF;
    vector<kfptr> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,kfptr> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        if(si>minScoreToRetain)
        {
            kfptr pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

void KeyFrameDatabase::AddMP(mpptr pMP)
{
    if(!pMP)
        return;

    unique_lock<mutex> lock(mMutexMPs);

    mmpMPs[pMP->mId] = pMP;
}

void KeyFrameDatabase::AddDirectBad(size_t id, size_t cid)
{
    unique_lock<mutex> lock(mMutexMPs);

    idpair idp = make_pair(id,cid);
    mmbDirectBad[idp] = true;
}

bool KeyFrameDatabase::FindMP(size_t id, size_t cid)
{
    unique_lock<mutex> lock(mMutexMPs);

    idpair idp = make_pair(id,cid);
    std::map<idpair,mpptr>::iterator mit = mmpMPs.find(idp);
    if(mit != mmpMPs.end()) return true;
    else return false;
}

bool KeyFrameDatabase::FindDirectBad(size_t id, size_t cid)
{
    unique_lock<mutex> lock(mMutexMPs);

    idpair idp = make_pair(id,cid);
    std::map<idpair,bool>::iterator mit = mmbDirectBad.find(idp);
    if(mit != mmbDirectBad.end()) return true;
    else return false;
}

void KeyFrameDatabase::ResetMPs()
{
    unique_lock<mutex> lock(mMutexMPs);

    mmbDirectBad.clear();
    mmpMPs.clear();
}

} //end ns
