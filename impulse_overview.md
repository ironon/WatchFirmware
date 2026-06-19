# Impulse — Master Product & Marketing Overview

*A single reference describing what Impulse is, what every part of it does, how the pieces fit together, and how to talk about it. This is the high-level companion to the detailed firmware specification; where the spec tells an engineer how to build it, this document explains what it is and why it matters.*

---

## 1. What Impulse Is

Impulse is a habit-enforcement system for people who already know what they want to do and can't reliably make themselves do it. It is built first and foremost for adults with ADHD, but it speaks to anyone who is tired of breaking promises to themselves.

The core idea is **self-binding**: you decide, in a calm and clear-headed moment, what kind of day you want — when to wake, when to move, when to put the phone down — and the system holds you to those decisions later, when a tireder, more impulsive version of you would rather renegotiate. Impulse is not a coach, a tracker, or a source of motivation. It is the mechanism that makes your own earlier decisions stick.

The system has three parts: a **watch** you wear, **anchors** you place around your home, and a **mobile app** where you design everything. They work together to enforce commitments tied to time, location, and proximity.

---

## 2. The Three Components

### 2.1 The Watch

The watch is the wearable core of the system. It is the device that knows the schedule, tracks where you are relative to your anchors and WiFi networks, and delivers the enforcement when a commitment isn't being met.

What it does:
- Holds your daily schedule and wakes itself at the start and end of each commitment window, sleeping in between to conserve battery.
- During a commitment window, checks whether you're meeting the criteria — near or away from a specific room, on or off a specific WiFi network — and escalates through vibration and sound if you're not.
- Knows when it's being worn, using a capacitive touch sensor. Taking the watch off during an active commitment doesn't end enforcement; it hands enforcement over to the anchors instead.
- Charges overnight and powers the system's signature wake-up experience (see Sunrise Lock, Section 3).
- Runs roughly 8–12 hours on a charge and refills in 2–3 hours, making overnight charging the natural rhythm.

What to call it in plain language: *the watch keeps you to the line you drew.* Avoid describing it as "enforcing rules" — it enforces *your own earlier decisions*.

### 2.2 The Anchors

Anchors are small devices you place at the locations tied to your habits: the nightstand, the desk, the front door. They are what lets Impulse work in the physical world rather than just on a phone screen.

What they do:
- Broadcast a consistent identity so the watch can tell which room it's in and how close it is.
- Sound an alarm when the watch is removed during a commitment window, in the rooms that matter — so walking away from the watch is the loud choice, not the easy one.
- Optionally lock down physically during an active window (an optional hardware clamp), so an anchor can't be quietly moved or muffled on impulse.
- Stay powered from the wall but carry an internal battery, so unplugging one buys no escape.
- Serve as a docking point for the phone during phone-distance commitments (see Section 3).

What to call them in plain language: *the part of Impulse that makes your commitments physical, and the reason you can't quietly opt out.* Be honest that they are hard to bypass *on impulse* — not "impossible to cheat." Overclaiming invites people to disprove you.

### 2.3 The Mobile App

The app is the calm side of the system — the place where clear-headed you sets everything up and then hands the day off to the hardware.

What it does:
- A visual weekly schedule builder where you lay out commitments: a time window, a place or network, and a firmness level.
- Per-commitment firmness, from a gentle nudge you can shrug off to a wake-up you can't.
- Anchor naming and placement guidance, including identifying which physical anchor is which (tap an anchor in the app and it beeps).
- The ability to disable a window *in advance*, before it starts, so a sick day or an early wake-up is never a fight.
- An emergency-pass system for the days you can't plan for, on a frequency you set.
- Live status: which anchors are online, where the watch is, whether it's being worn, battery level.

A critical design principle runs through the app: **changes to enforcement settings take roughly a day to take effect.** You can always make the system *easier* on yourself in advance, but you cannot grant yourself an escape in the heat of the moment. That delay is not a limitation — it is the feature. It's the difference between a real exception and a 6am excuse.

What to call it in plain language: *where clear-headed you runs the show.*

---

## 3. The Signature Experiences

These are the two features that best define the product and should anchor most marketing.

### 3.1 Sunrise Lock

The flagship demo. The watch charges overnight on your nightstand, next to its anchor. At your wake-up time, the anchor begins to sound while the watch sits charged and ready. Putting the watch on quiets the room for a short grace period you set yourself. If you're still in bed when the grace period ends, the watch vibrates and the anchor sounds together, escalating, until you're up and out of the room.

Why it matters: it is the alarm you can't snooze, because the alarm isn't on your nightstand — it's the whole room. It directly solves the single most relatable problem in the category (getting out of bed) with a mechanism that obviously works, and every parameter is something you set the night before. Past-you runs the morning.

### 3.2 Phone-Distance Commitments

This is how Impulse addresses screen time, and it's deliberately *not* a copy of Opal or other app-blockers.

Most screen-time blockers share a fatal flaw: the enforcer is software on the same phone you're trying to control, so the moment you really want to scroll, you disable it and win. Impulse takes a different path entirely. Instead of blocking individual apps, it enforces **physical distance from the phone**. You set windows in the app during which your phone must be docked near a specific anchor (or, secondarily, far from you). If you spend too long near your phone during one of these windows — past a configurable tolerance, so a quick check is fine — the watch and anchors sound until you put it back down.

This is a stronger story than app-blocking: *we don't block the distracting apps, we keep the phone out of your hands entirely.*

**Important honesty note for all materials:** Impulse no longer blocks apps and does not detect uninstalls. Earlier concepts explored both; neither survived contact with what the platforms actually allow. Do not market app-blocking or uninstall-detection. Do not leave stale "blocks distracting apps" or "cannot be bypassed by uninstalling the app" claims in any comparison material — those capabilities were cut, and claiming them is both untrue and an invitation to be disproven.

**Reliability framing for this feature:** enforcement should follow the quality of the device link, not a battery percentage — if the connection is solid, enforce; if it's degraded, fail *open* (don't enforce) rather than risk a false alarm. Lead users toward the anchor-docking setup, which is reliable, rather than the phone-far-from-watch setup, which is inherently noisier. Be upfront in setup guidance that the phone must be docked near the anchor, the app left running, and the phone kept out of low power mode for the feature to behave.

---

## 4. How It All Fits Together

A single day shows the system working as a whole:

The night before, you charge the watch on your nightstand. At 6am, Sunrise Lock gets you up and out of the bedroom. From 6:30 to 9, a phone-distance window keeps your phone docked at the kitchen anchor so your morning isn't lost to scrolling. At 9, a commitment to be at your desk begins, verified by the office anchor. At 5:30, a window enforces that you've left home and arrived on your gym's WiFi. At 9pm, another phone-distance window winds the day down. Each of these is something you decided once, in the app, and then stopped having to think about.

The throughline: **you design the shape of the day, and then you walk into the shape you built.** The hardware carries the decisions so you don't have to keep re-making them.

---

## 5. Positioning & Competitive Frame

Impulse sits near three existing categories but doesn't belong to any of them:

- **Habit apps** track what you do and trust you to keep going. The trust is exactly where they fail — nothing happens when you ignore them.
- **Phone blockers** block apps but live on the phone they're policing, so a determined user always wins.
- **Shock-based wearables** deliver physical feedback but only break habits; they don't follow you into the physical world or tie to real locations.

Impulse's distinct ground: it meets you in the physical world, at the moment of the decision, tied to real places and real proximity, and it can't be quietly opted out of mid-commitment. The single sharpest differentiator is that **the enforcer isn't the thing being bypassed** — take off the watch and the anchors take over; dock the phone and the distance is what's measured, not an app you can delete.

When building any comparison table, the honest column for Impulse now includes: real-world location enforcement, phone-proximity enforcement, removal detection, anchor hand-off, and physical lockdown — and *excludes* app-blocking. Keep it honest; an accurate table is more persuasive than an inflated one.

---

## 6. Audience

The primary audience is **adults with ADHD**, who will recognize immediately the thing Impulse takes off their plate: the exhausting, all-day project of managing themselves into doing what they already wanted to do. Foreground ADHD enough that these users feel seen.

But the product is not ADHD-exclusive, and the messaging shouldn't make it feel that way. The secondary audiences — people fixing their sleep, people trying to get to the gym, students, remote workers, parents trying to be present, creatives trying to show up to the work — all share the same underlying problem: the gap between intention and action. You don't need a diagnosis to be tired of breaking your own promises.

A note of care: in any materials that might reach people in recovery or experiencing depression, tread gently. The same lever feels very different to someone in crisis, and these audiences are best reached through partnership with professionals rather than direct marketing. Avoid any language implying medical efficacy.

---

## 7. Voice & Language Guide

The single most important rule: **sell the freedom on the other side of the friction, not the friction itself.** The buzzing and beeping are the mechanism, not the brand. The brand is the calm, the headspace, and the time you get back when you stop negotiating with yourself.

### Words and frames to use

- **Self-binding language.** The system honors *your own earlier decision*. "You decided this, this morning." "Past-you runs the day." This is the frame that turns every "isn't this harsh?" reaction into "oh, I'm doing this to myself, on purpose."
- **Freedom and relief.** "Stop negotiating with yourself." "Get your mornings back." "The quiet that comes when you finally stop fighting yourself."
- **Agency.** The user is the expert on their own life. They design the day; the hardware carries it out.
- **Plain, concrete specifics.** "Up and out of the bedroom by 6:30," "your gym's WiFi confirms you made it." Concrete beats clever every time.
- **Honest confidence.** "Hard to beat on impulse," not "impossible to cheat." Honesty is more persuasive and more defensible.

### Words and frames to avoid

- **The word "enforcement"** in consumer-facing copy. Internally it's fine; to a buyer it sounds like punishment. Use *accountability, follow-through, commitment, holds the line* instead — and even those, sparingly.
- **Parole-officer language.** Avoid leaning on "the watch holds you to..." as a repeated drumbeat. Flip it to the user's own authorship wherever possible.
- **Mechanism-worship.** Don't make relentlessness the personality. Describe the buzzing factually where users need to know it (FAQ, how-it-works), but don't headline it.
- **Coercive flourishes.** Lines like "it doesn't care that it's cold outside" are memorable but sell exactly the wrong thing.
- **Overclaiming.** No "impossible to cheat," no medical-efficacy claims, no promising app-blocking or uninstall-detection (both cut).
- **AI cadence.** Watch for the relentless triple-beat rhythm (short fragment, elaboration, punchy reversal) repeated paragraph after paragraph. Use it once or twice for emphasis; vary sentence length everywhere else. Read copy aloud — if you hear the same drumbeat twice in a row, rewrite one.

### Taglines in play

- *Stop negotiating with yourself.* (primary)
- *Keep the promises you make to yourself.*
- *Outsource your willpower.*

---

## 8. Honest Caveats to Keep Front of Mind

These are the things most likely to generate frustrated customers if mishandled, and they should be addressed openly rather than hidden:

- **Phone-distance reliability.** Low power mode and degraded Bluetooth links can cause false alarms. Mitigate by failing open on poor link quality, steering users to anchor-docking, and being clear in setup about the phone conditions required.
- **The anchor-lock and muffling.** A determined user can sometimes still defeat an anchor (e.g. muffling it). Market it as hard-to-beat-on-impulse, never as impossible.
- **Overnight battery.** The watch doesn't last multiple days; the overnight-charging model must be communicated clearly so buyers don't expect always-on wear.
- **Capabilities that were cut.** App-blocking and uninstall-detection are gone. Keep all materials consistent with that.

---

*This document is a living overview and should be updated as the product evolves. For implementation-level detail, refer to the firmware specification.*
