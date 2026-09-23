from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
NOTE = ROOT / "NEXT_CHANGES.md"


def main() -> None:
    text = NOTE.read_text(encoding="utf-8")

    required = {
        "active deterministic-fusion verdict": "useful deterministic fusion",
        "current one-second throttle": "1000 ms",
        "camera integration truth": "fixed PC camera can correct the along-wall Y shown in the web UI",
        "fresh-observation policy": "every fresh",
        "estimator uncertainty": "estimator uncertainty",
        "measurement uncertainty": "measurement uncertainty",
        "normalized innovation": "normalized innovation",
        "LiDAR observer": "## LiDAR Wall Observer",
        "camera observer": "## AI Vision AprilTag Observer",
        "duplicate-ID handling": "Duplicate-ID Association",
        "shadow rollout": "shadow mode",
        "prediction-only mode": "`PREDICT_ONLY`",
        "recovery mode": "`RECOVERY`",
        "90-second evaluation": "90-second",
        "future implementation status": "not the proposed covariance",
        "no large-residual trust rule": (
            "Do not trust LiDAR or AI Vision automatically because the difference is large."
        ),
    }

    missing = [name for name, phrase in required.items() if phrase not in text]
    if missing:
        raise AssertionError("Missing required design content: " + ", ".join(missing))

    obsolete = {
        "old vertical-wheel target": "vertical tracking wheel = real forward distance",
        "obsolete vertical-wheel priority": "Add a vertical odom wheel if mechanically possible",
        "old document title": "# Next Localization Changes",
    }
    found_obsolete = [name for name, phrase in obsolete.items() if phrase in text]
    if found_obsolete:
        raise AssertionError("Obsolete content remains: " + ", ".join(found_obsolete))

    if len(text.splitlines()) < 250:
        raise AssertionError("Master-fusion design is unexpectedly short")

    print("PASS: NEXT_CHANGES.md fully describes the master fusion design")


if __name__ == "__main__":
    main()
