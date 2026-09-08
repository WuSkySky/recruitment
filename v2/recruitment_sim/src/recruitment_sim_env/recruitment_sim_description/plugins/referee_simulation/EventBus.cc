#include "EventBus.hh"
#include <mutex>
#include <set>
namespace recruitment_sim
{
namespace {
std::mutex mutex;
uint64_t round_id = 0;
std::vector<std::string> events;
std::set<uint64_t> shooters, projectiles;
}
void RecordEvent(const std::string & event) {std::lock_guard<std::mutex> l(mutex); events.push_back(event);}
std::vector<std::string> DrainEvents() {std::lock_guard<std::mutex> l(mutex); std::vector<std::string> out; out.swap(events); return out;}
uint64_t Round() {std::lock_guard<std::mutex> l(mutex); return round_id;}
void SetRound(uint64_t round) {std::lock_guard<std::mutex> l(mutex); round_id = round; events.clear();}
void RegisterShooter(uint64_t e) {std::lock_guard<std::mutex> l(mutex); shooters.insert(e);}
bool ShooterReady(uint64_t e) {std::lock_guard<std::mutex> l(mutex); return shooters.count(e) != 0;}
void RegisterProjectile(uint64_t e) {std::lock_guard<std::mutex> l(mutex); projectiles.insert(e);}
void ForgetProjectile(uint64_t e) {std::lock_guard<std::mutex> l(mutex); projectiles.erase(e);}
std::vector<uint64_t> TakeProjectiles() {std::lock_guard<std::mutex> l(mutex); std::vector<uint64_t> out(projectiles.begin(), projectiles.end()); projectiles.clear(); return out;}
}
