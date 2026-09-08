#pragma once
#include <cstdint>
#include <string>
#include <vector>

// Shared by the shooter and world libraries in one Gazebo server. Shooter
// PreUpdate writes; world PostUpdate drains a complete step (no transport race).
namespace recruitment_sim
{
void RecordEvent(const std::string & event);
std::vector<std::string> DrainEvents();
uint64_t Round();
void SetRound(uint64_t round);
void RegisterShooter(uint64_t entity);
bool ShooterReady(uint64_t entity);
void RegisterProjectile(uint64_t entity);
std::vector<uint64_t> TakeProjectiles();
void ForgetProjectile(uint64_t entity);
}
