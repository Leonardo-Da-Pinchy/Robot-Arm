const button = document.querySelector("#check-backend");
const status = document.querySelector("#status");

button.addEventListener("click", async () => {
  try {
    const response = await fetch(button.dataset.url);
    if (!response.ok) throw new Error("Request failed");
    const data = await response.json();
    status.textContent = data.message;
  } catch {
    status.textContent = "Could not connect to the backend.";
  }
});
