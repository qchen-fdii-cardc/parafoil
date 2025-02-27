document.addEventListener('DOMContentLoaded', function() {
    const imgs = document.querySelectorAll('img');
    imgs.forEach(img => {
        if (img.width > 400) img.width = 400;
        if (img.height > 400) img.height = 400;
        
        const title = document.createElement('div');
        title.style.textAlign = 'center';
        title.style.marginTop = '10px';
        title.textContent = img.title || img.alt;
        img.parentNode.insertBefore(title, img.nextSibling);
    });
}); 