export default {
  async fetch(request, env, ctx) {
    // Only accept POST requests
    if (request.method !== "POST") {
      return new Response("Method not allowed", { status: 405 });
    }

    // Authenticate the request using a shared secret.
    // Header name and Cloudflare env var name are both "BUOY_SECRET" by
    // convention here so the Hologram alert config matches the worker config
    // without a separate X-Buoy-Secret naming.
    const secret = request.headers.get("BUOY_SECRET");
    if (!env.BUOY_SECRET || secret !== env.BUOY_SECRET) {
      return new Response("Unauthorized", { status: 401 });
    }

    try {
      // Parse the incoming JSON telemetry from the buoy
      const buoyData = await request.json();

      // Check if required environment variables are set
      if (!env.GITHUB_PAT || !env.GITHUB_OWNER || !env.GITHUB_REPO) {
        return new Response("Worker is missing GitHub configuration.", { status: 500 });
      }

      // Construct the payload for GitHub repository_dispatch
      const githubPayload = {
        event_type: "buoy-telemetry",
        client_payload: buoyData
      };

      // Forward to GitHub API
      const githubResponse = await fetch(`https://api.github.com/repos/${env.GITHUB_OWNER}/${env.GITHUB_REPO}/dispatches`, {
        method: "POST",
        headers: {
          "Accept": "application/vnd.github+json",
          "Authorization": `Bearer ${env.GITHUB_PAT}`,
          "X-GitHub-Api-Version": "2022-11-28",
          "User-Agent": "Cloudflare-Worker-Buoy-Proxy"
        },
        body: JSON.stringify(githubPayload)
      });

      if (!githubResponse.ok) {
        const errorText = await githubResponse.text();
        return new Response(`GitHub API error: ${githubResponse.status} - ${errorText}`, { status: 502 });
      }

      return new Response("Successfully forwarded to GitHub Actions", { status: 200 });

    } catch (err) {
      return new Response(`Bad Request: ${err.message}`, { status: 400 });
    }
  },
};
