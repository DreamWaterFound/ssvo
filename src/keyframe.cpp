#include "config.hpp"
#include "map.hpp"
#include "keyframe.hpp"

namespace ssvo{

uint64_t KeyFrame::next_id_ = 0;

KeyFrame::KeyFrame(const Frame::Ptr frame):
    Frame(frame->images(), next_id_++, frame->timestamp_, frame->cam_), frame_id_(frame->id_), isBad_(false), loop_query_(0),
    notErase(false),toBeErase(false),GBA_KF_(0)
{
    mpt_fts_ = frame->features();
    setRefKeyFrame(frame->getRefKeyFrame());
    setPose(frame->pose());
}
void KeyFrame::updateConnections()
{
    if(isBad())
        return;

    Features fts;
    {
        std::lock_guard<std::mutex> lock(mutex_feature_);
        for(const auto &it : mpt_fts_)
            fts.push_back(it.second);
    }

    std::map<KeyFrame::Ptr, int> connection_counter;

    for(const Feature::Ptr &ft : fts)
    {
        const MapPoint::Ptr &mpt = ft->mpt_;

        if(mpt->isBad())
        {
            removeFeature(ft);
            continue;
        }

        const std::map<KeyFrame::Ptr, Feature::Ptr> observations = mpt->getObservations();
        for(const auto &obs : observations)
        {
            if(obs.first->id_ == id_)
                continue;
            connection_counter[obs.first]++;
        }
    }

    if(connection_counter.empty())
    {
        setBad();
        return;
    }

    // TODO how to select proper connections
    int connection_threshold = Config::minConnectionObservations();

    KeyFrame::Ptr best_unfit_keyframe;
    int best_unfit_connections = 0;
    std::vector<std::pair<int, KeyFrame::Ptr> > weight_connections;
    for(const auto &obs : connection_counter)
    {
        if(obs.second < connection_threshold)
        {
            best_unfit_keyframe = obs.first;
            best_unfit_connections = obs.second;
        }
        else
        {
            obs.first->addConnection(shared_from_this(), obs.second);
            weight_connections.emplace_back(std::make_pair(obs.second, obs.first));
        }
    }

    if(weight_connections.empty())
    {
        best_unfit_keyframe->addConnection(shared_from_this(), best_unfit_connections);
        weight_connections.emplace_back(std::make_pair(best_unfit_connections, best_unfit_keyframe));
    }

    //! sort by weight
    std::sort(weight_connections.begin(), weight_connections.end(),
              [](const std::pair<int, KeyFrame::Ptr> &a, const std::pair<int, KeyFrame::Ptr> &b){ return a.first > b.first; });

    //! update
    {
        std::lock_guard<std::mutex> lock(mutex_connection_);

        connectedKeyFrames_.clear();
        for(const auto &item : weight_connections)
        {
            connectedKeyFrames_.insert(std::make_pair(item.second, item.first));
        }

        orderedConnectedKeyFrames_ =
            std::multimap<int, KeyFrame::Ptr>(weight_connections.begin(), weight_connections.end());
    }
}

std::set<KeyFrame::Ptr> KeyFrame::getConnectedKeyFrames(int num, int min_fts)
{
    std::lock_guard<std::mutex> lock(mutex_connection_);

    std::set<KeyFrame::Ptr> connected_keyframes;
    if(num == -1) num = (int) orderedConnectedKeyFrames_.size();

    int count = 0;
    const auto end = orderedConnectedKeyFrames_.rend();
    for(auto it = orderedConnectedKeyFrames_.rbegin(); it != end && it->first >= min_fts && count < num; it++, count++)
    {
        connected_keyframes.insert(it->second);
    }

    return connected_keyframes;
}

std::set<KeyFrame::Ptr> KeyFrame::getSubConnectedKeyFrames(int num)
{
    std::set<KeyFrame::Ptr> connected_keyframes = getConnectedKeyFrames();

    std::map<KeyFrame::Ptr, int> candidate_keyframes;
    for(const KeyFrame::Ptr &kf : connected_keyframes)
    {
        std::set<KeyFrame::Ptr> sub_connected_keyframe = kf->getConnectedKeyFrames();
        for(const KeyFrame::Ptr &sub_kf : sub_connected_keyframe)
        {
            if(connected_keyframes.count(sub_kf) || sub_kf == shared_from_this())
                continue;

            if(candidate_keyframes.count(sub_kf))
                candidate_keyframes.find(sub_kf)->second++;
            else
                candidate_keyframes.emplace(sub_kf, 1);
        }
    }

    std::set<KeyFrame::Ptr> sub_connected_keyframes;
    if(num == -1)
    {
        for(const auto &item : candidate_keyframes)
            sub_connected_keyframes.insert(item.first);

        return sub_connected_keyframes;
    }

    //! stort by order
    std::map<int, KeyFrame::Ptr, std::greater<int> > ordered_candidate_keyframes;
    for(const auto &item : candidate_keyframes)
    {
        ordered_candidate_keyframes.emplace(item.second, item.first);
    }

    //! get best (num) keyframes
    for(const auto &item : ordered_candidate_keyframes)
    {
        sub_connected_keyframes.insert(item.second);
        if(sub_connected_keyframes.size() >= num)
            break;
    }

    return sub_connected_keyframes;
}

void KeyFrame::setNotErase()
{
    std::unique_lock<std::mutex> lock(mutex_connection_);
    notErase = true;
}

//! 这函数在闭环的时候常用，因为开始闭环的时候设了卡
void KeyFrame::setErase()
{
    {
        //todo 2 add loop detect finish condition
        std::unique_lock<std::mutex> lock(mutex_connection_);
        if(false)
        {
            notErase = false;
        }
    }
    if(toBeErase)
    {
        setBad();
    }
}

void KeyFrame::setBad()
{

    {
        std::unique_lock<std::mutex> lock(mutex_connection_);
        if(id_ == 0)
            return;
        //! 这里是如果想删除，就等一等，设置toBeErase变量，等闭环结束之后会调用keyframe的seterase，如果想删除的话那会再删除
        if(notErase)
        {
            toBeErase = true;
            return;
        }
    }

    std::cout << "The keyframe " << id_ << " was set to be earased." << std::endl;

    std::unordered_map<MapPoint::Ptr, Feature::Ptr> mpt_fts;
    {
        std::lock_guard<std::mutex> lock(mutex_feature_);
        mpt_fts = mpt_fts_;
    }

    for(const auto &it : mpt_fts)
    {
        it.first->removeObservation(shared_from_this());
    }

    {
        std::lock_guard<std::mutex> lock(mutex_connection_);

        isBad_ = true;

        for(const auto &connect : connectedKeyFrames_)
        {
            connect.first->removeConnection(shared_from_this());
        }

        connectedKeyFrames_.clear();
        orderedConnectedKeyFrames_.clear();
        mpt_fts_.clear();
        seed_fts_.clear();
    }
    // TODO change refKF
}

bool KeyFrame::isBad()
{
    std::lock_guard<std::mutex> lock(mutex_connection_);
    return isBad_;
}

void KeyFrame::addConnection(const KeyFrame::Ptr &kf, const int weight)
{
    {
        std::lock_guard<std::mutex> lock(mutex_connection_);

        if(!connectedKeyFrames_.count(kf))
            connectedKeyFrames_[kf] = weight;
        else if(connectedKeyFrames_[kf] != weight)
            connectedKeyFrames_[kf] = weight;
        else
            return;
    }

    updateOrderedConnections();
}

void KeyFrame::updateOrderedConnections()
{
    int max = 0;
    std::lock_guard<std::mutex> lock(mutex_connection_);
    orderedConnectedKeyFrames_.clear();
    for(const auto &connect : connectedKeyFrames_)
    {
        auto it = orderedConnectedKeyFrames_.lower_bound(connect.second);
        orderedConnectedKeyFrames_.insert(it, std::pair<int, KeyFrame::Ptr>(connect.second, connect.first));

        if(connect.second > max)
        {
            max = connect.second;
            parent_ = connect.first;
        }
    }
}

void KeyFrame::removeConnection(const KeyFrame::Ptr &kf)
{
    {
        std::lock_guard<std::mutex> lock(mutex_connection_);
        if(connectedKeyFrames_.count(kf))
        {
            connectedKeyFrames_.erase(kf);
        }
    }

    updateOrderedConnections();
}

std::vector<int > KeyFrame::getFeaturesInArea(const double &x, const double &y, const double &r)
{
    std::vector<int > index;

    for(int i = 0; i < featuresInBow.size(); ++i)
    {
        Feature::Ptr it = featuresInBow[i];
        if( it->px_[0] < (x-r) || it->px_[0] > (x+r) || it->px_[1] < (y-r) || it->px_[1] > (y + r))
            continue;
        if(((it->px_[0]- x)*(it->px_[0]- x)+(it->px_[1]- y)*(it->px_[1]- y)) < (double)r*r)
            index.push_back(i);
    }
    return index;
}

void KeyFrame::addLoopEdge(KeyFrame::Ptr pKF)
{
    std::unique_lock<std::mutex > lock(mutex_connection_);
    notErase = true;
    loopEdges_.insert(pKF);
}

int KeyFrame::getWight(KeyFrame::Ptr pKF)
{
    std::lock_guard<std::mutex> lock(mutex_connection_);
    return connectedKeyFrames_[pKF];
}
KeyFrame::Ptr KeyFrame::getParent()
{
    return parent_;
}

std::set<KeyFrame::Ptr> KeyFrame::getLoopEdges()
{
    std::lock_guard<std::mutex> lock(mutex_connection_);
    return loopEdges_;

}

}